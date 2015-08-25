#include "chessBoardCornerDetector.h"

#include <cmath>
#include <cassert>
#include <algorithm>
#include <set>

double OrientedCorner::scoreCorner(DpImage &img, DpImage &weight, std::vector<double> &radius, double bandwidth)
{
    int iw = img.w, ih = img.h;
    score = 0.0;
    for (auto& r: radius)
    {
        if (!(pos[0] - r >= 0 && pos[0] + r < iw && pos[1] -r >= 0 && pos[1] + r < ih))
            continue;

        double local = scoreCorner(img, weight, r, bandwidth);
        if (local > score)
            score = local;
    }
    return score;
}

double OrientedCorner::scoreCorner(DpImage &img, DpImage &weight, int r, double bandwidth)
{
    return scoreGradient(weight, r, bandwidth) * scoreIntensity(img, r);
}

double OrientedCorner::scoreGradient(DpImage &w, int r, double bandwidth)
{
    int kw = 2 * r + 1;
    DpImage K(kw, kw);
    for (int i = 0; i < kw; ++i)
    {
        for (int j = 0; j < kw; ++j)
        {
            K.element(i, j) = -1.0;
        }
    }
    double K_sum = -kw * kw, K_sum_sq = kw * kw;
    double I_sum = 0.0, I_sum_sq = 0.0;
    int cx = r, cy = r;
    int ui = pos[0], vi = pos[1];
    corecvs::Vector2dd c(r, r);
    for (int i = 0; i < kw; ++i)
    {
        for (int j = 0; j < kw; ++j)
        {
            corecvs::Vector2dd p(j, i);
            auto pp = p - c;
            auto p1 = (pp & v1) * v1;
            auto p2 = (pp & v2) * v2;

            if ((pp - p1).l2Metric() < bandwidth / 2.0 || (pp - p2).l2Metric() < bandwidth / 2.0)
            {
                K.element(i, j) = 1.0;
                K_sum += 2.0;
            }

            double val =  w.element(vi + i - cy, ui + j - cx);
            I_sum += val;
            I_sum_sq += val * val;
        }
    }

    double mean_w = I_sum / (kw * kw);
    double  std_w = std::sqrt(I_sum_sq / (kw * kw) - mean_w * mean_w);
    double mean_k = K_sum / (kw * kw);
    double  std_k = std::sqrt(K_sum_sq / (kw * kw) - mean_k * mean_k);

    double score = 0.0;
    for (int i = 0; i < kw; ++i)
    {
        for (int j = 0; j < kw; ++j)
        {
            score += (K.element(i, j) - mean_k) / std_k * (w.element(vi + i - cy, ui + j - cx) - mean_w) / std_w;
        }
    }
    return std::max(score / (kw * kw), 0.0);
}

double OrientedCorner::scoreIntensity(DpImage &img, int r)
{
    int w = (r + 1) * 2 + 1;
    double alpha = atan2(v1[1], v1[0]);
    double psi = atan2(v2[1], v2[0]) - alpha;
    CornerKernelSet cks(r, alpha, psi);
    DpImage patch(w, w);
    int uc = pos[0], vc = pos[1];
    int left = std::max(0, r - uc);
    int right = std::min(w, img.w + r - uc);
    int top = std::max(0, r - vc);
    int bottom = std::min(w, img.h + r - vc);
    for (int i = 0; i < w; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            if (i >= top && i < bottom && j >= left && j < right)
                patch.element(i, j) = img.element(i - r + vc, j - r + uc);
            else
                patch.element(i, j) = 0.0;
        }
    }

    DpImage c(w, w);
    //int cc = cks.A.x;
    cks.computeCost(patch, c, true);
    return std::max(c.element(cks.A.y, cks.A.x), 0.0);
}

CornerKernelSet::CornerKernelSet(double r, double alpha, double psi)
{
    int w = ((int)(r + 1.0)) * 2 + 1;
    A = DpKernel(w, w);
    B = DpKernel(w, w);
    C = DpKernel(w, w);
    D = DpKernel(w, w);

    int c = w / 2;

    computeKernels(r, alpha, psi, w, c);
}

void CornerKernelSet::computeCost(DpImage &img, DpImage &c, bool parallelable)
{
    int w = img.w, h = img.h;
    c = DpImage(h, w);

    auto *pfA = img.doConvolve<DpImage>(&A, true, parallelable);
    auto *pfB = img.doConvolve<DpImage>(&B, true, parallelable);
    auto *pfC = img.doConvolve<DpImage>(&C, true, parallelable);
    auto *pfD = img.doConvolve<DpImage>(&D, true, parallelable);

    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            double fA = pfA->element(i, j),
                   fB = pfB->element(i, j),
                   fC = pfC->element(i, j),
                   fD = pfD->element(i, j);

            double mu = 0.25 * (fA + fB + fC + fD);
            double a = std::min(fA - mu, fB - mu);
            double b = std::min(mu - fC, mu - fD);
            double r1 = std::min(a, b);
            a = std::min(fC - mu, fD - mu);
            b = std::min(mu - fA, mu - fB);
            double r2 = std::min(a, b);


            c.element(i, j) = std::max(r1, r2);
        }
    }

    // FIXME: we should definitely change convolution routine
    delete pfA;
    delete pfB;
    delete pfC;
    delete pfD;
}

void CornerKernelSet::computeKernels(double r, double alpha, double psi, int w, int c, double threshold)
{
    auto v1 = corecvs::Vector2dd(cos(alpha),       sin(alpha)).rightNormal();
    auto v2 = corecvs::Vector2dd(cos(alpha + psi), sin(alpha + psi)).rightNormal();
    corecvs::Vector2dd cc(c, c);

    //double sum = 0.0;
    for (int i = 0; i < w; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            corecvs::Vector2dd p(i, j);
            auto v = p - cc;
            // Now we check if it is inside => cross product changes sign
            auto p1 = v & v1, p2 = v &v2;
            double lr = !v, sigma = r / 2.0;

            A.element(i, j) = B.element(i, j) = C.element(i, j) = D.element(i, j) = 0.0;
            if (p1 > threshold && p2 < -threshold)
            {
                A.element(i, j) = pdf(lr, sigma);
            }
            if (p1 < -threshold && p2 > threshold)
            {
                B.element(i, j) = pdf(lr, sigma);
            }
            if (p1 < -threshold && p2 < -threshold)
            {
                C.element(i, j) = pdf(lr, sigma);
            }
            if (p1 > threshold && p2 > threshold)
            {
                D.element(i, j) = pdf(lr, sigma);
            }
        }
    }
#if 0
    std::cout << "[" << std::endl;
    for (int i = 0; i < w; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            std::cout << A.element(i, j) << " ";
        }
        std::cout << ";" << std::endl;
    }
    for (int i = 0; i < w; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            std::cout << B.element(i, j) << " ";
        }
        std::cout << ";" << std::endl;
    }
    for (int i = 0; i < w; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            std::cout << C.element(i, j) << " ";
        }
        std::cout << ";" << std::endl;
    }
    for (int i = 0; i < w; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            std::cout << D.element(i, j) << " ";
        }
        std::cout << ";" << std::endl;
    }
    std::cout << "]" << std::endl;
    exit(0);
#endif
}

void ChessBoardCornerDetector::prepareDiff(DpImage &diff, bool du)
{
    double dvk[] = {-1.0, -1.0, -1.0,  0.0, 0.0, 0.0,  1.0, 1.0, 1.0};
    double duk[] = {-1.0,  0.0,  1.0, -1.0, 0.0, 1.0, -1.0, 0.0, 1.0};
    DpKernel K (3, 3, du ? duk : dvk);

    auto *blah_noway = img.doConvolve<DpImage>(&K);
    diff = *blah_noway;
    delete blah_noway;
}

#define SQR
void ChessBoardCornerDetector::prepareAngleWeight()
{
    int W = du.w;
    int H = du.h;

    phi = DpImage(H, W);
    w   = DpImage(H, W);

    for (int i = 0; i < H; ++i)
    {
        for (int j = 0; j < W; ++j)
        {
            double Du = du.element(i, j);
            double Dv = dv.element(i, j);
            double Phi = atan2(Dv, Du) + M_PI / 2.0;
            if (Phi > M_PI) Phi -= M_PI;
            if (Phi < 0.0) Phi += M_PI;
            phi.element(i, j) = Phi;
            w.element(i, j) = sqrt(Dv * Dv + Du * Du);
        }
    }
}
#undef SQR

// XXX: due to distortion removal we can get some black areas.
// Let us scale image based on 0.05 and 0.95 percentiles
void ChessBoardCornerDetector::scaleImage()
{
    std::vector<double> values;
    for (int i = 0; i < img.h; ++i)
    {
        for (int j = 0; j < img.w; ++j)
        {
            values.push_back(img.element(i, j));
        }
    }

    std::sort(values.begin(), values.end());

    double p05 = values[0.05 * values.size()];
    double p95 = values[0.95 * values.size()];
    double dp = p95 - p05;

    for (int i = 0; i < img.h; ++i)
    {
        for (int j = 0; j < img.w; ++j)
        {
            img.element(i, j) = std::max(std::min((img.element(i, j) - p05) / dp, 1.0), 0.0);
        }
    }

}


void ChessBoardCornerDetector::prepareKernels()
{
    kernels.clear();

    for (auto& r: patternRadius)
    {
        for (auto& psi: patternStartAngle)
        {
            kernels.emplace_back(r, psi, sectorSize);
        }
    }
}

void ChessBoardCornerDetector::computeCost()
{
    int w = img.w, h = img.h;
    cost = DpImage(h, w);

    for (auto& cks: kernels)
    {
        DpImage kc;
        cks.computeCost(img, kc);

        cost.binaryOperationInPlace(kc, [](const double &a, const double &b) { return std::max(a, b); });
    }
}

void ChessBoardCornerDetector::runNms()
{
    std::vector<std::pair<int, int>> cornerCandidates;
    cost.nonMaximumSupression(5, 0.025, cornerCandidates, 5);
    for (auto& cc: cornerCandidates)
        corners.emplace_back(corecvs::Vector2dd(cc.first, cc.second));
}


void ChessBoardCornerDetector::circularMeanShift(std::vector<double> &values, double bandwidth, std::vector<std::pair<int, double>> &modes)
{
    int N = values.size();
    std::vector<double> smoothed(N);

    for (int i = 0; i < N; ++i)
    {
        double sum = 0.0;
        for (int j = -std::ceil(3.0 * bandwidth); j <= std::ceil(3.0 * bandwidth); ++j)
        {
            int idx = j + i;
            while(idx < 0) idx += N;
            sum += values[idx % N] * exp(-(j * j) / (2.0 * bandwidth * bandwidth)) / (bandwidth * sqrt(2.0 * M_PI));
        }
        smoothed[i] = sum;
    }

    bool ok = false;
    for (int i = 1; i < N && !ok; ++i)
    {
        if (smoothed[i] != smoothed[0])
        {
            ok = true;
        }
    }
    modes.clear();
    if (!ok)
        return;

    std::set<int> modes_set;

    for (int seed = 0; seed < N; ++seed)
    {
        int i = seed;
        while (1)
        {
            int left = (i + N - 1) % N;
            int right = (i + N + 1) % N;
            double val = smoothed[i], lval = smoothed[left], rval = smoothed[right];

            if (lval >= val && lval >= rval)
            {
                i = left;
                continue;
            }
            if (rval > val && rval > lval)
            {
                i = right;
                continue;
            }
            break;
        }
        if (!modes_set.count(i))
        {
            modes_set.insert(i);
        }
    }

    for (auto& m: modes_set)
    {
        modes.push_back(std::make_pair(m, smoothed[m]));
    }
}


bool ChessBoardCornerDetector::edgeOrientationFromGradient(int top, int bottom, int left, int right, corecvs::Vector2dd &v1, corecvs::Vector2dd &v2)
{
    std::vector<double> histogram(histogramBins);
    double bin_size = M_PI / histogramBins;

    for (int i = top; i <= bottom; ++i)
    {
        for (int j = left; j <= right; ++j)
        {
            double Phi = phi.element(i, j);
            double W  = w.element(i, j);

            int bin = std::max(std::min((int)(Phi / bin_size), histogramBins - 1), 0);
            histogram[bin] += W;
        }
    }

    typedef std::pair<int, double> PairID;
    std::vector<PairID > modes;
    circularMeanShift(histogram, meanshiftBandwidth, modes);

    if (modes.size() < 2)
    {
        return false;
    }

    //std::sort(modes.begin(), modes.end(), [](decltype(modes[0]) a, decltype(modes[0]) b) { return a.second == b.second ? a.first < b.first : a.second > b.second; });

    std::sort(modes.begin(), modes.end(), [](PairID a, PairID b) { return a.second == b.second ? a.first < b.first : a.second > b.second; });

    auto p1 = modes[0], p2 = modes[1];
    double phi1 = p1.first * bin_size, phi2 = p2.first * bin_size;
    if (phi1 > phi2)
        std::swap(phi1, phi2);

    double angle = std::min(phi2 - phi1, M_PI + phi1 - phi2);
    if (angle < minAngle)
    {
        return false;
    }

    v1 = corecvs::Vector2dd(cos(phi1), sin(phi1));
    v2 = corecvs::Vector2dd(cos(phi2), sin(phi2));
    return true;
}

void ChessBoardCornerDetector::filterByOrientation()
{
    int idx = 0, N = corners.size(), iw = w.w, ih = w.h;
    for (int i = 0; i < N; ++i)
    {
        auto& c = corners[i];

        int top = std::max(0.0, c.pos[1] - neighborhood);
        int bottom = std::min(ih - 1.0, c.pos[1] + neighborhood);
        int left = std::max(0.0, c.pos[0] - neighborhood);
        int right = std::min(iw - 1.0, c.pos[0] + neighborhood);

        if (edgeOrientationFromGradient(top, bottom, left, right, c.v1, c.v2))
        {
            corners[idx++] = c;
        }
    }
    corners.resize(idx);
}

void ChessBoardCornerDetector::eig22(corecvs::Matrix &A, double &lambda1, corecvs::Vector2dd &e1, double &lambda2, corecvs::Vector2dd &e2)
{
    const double EIGTOLERANCE = 1e-9;
    assert(A.w == 2 && A.h == 2);
    double T = A.a(0, 0) + A.a(1, 1);
    double D = A.a(0, 0) * A.a(1, 1) - A.a(1, 0) * A.a(0, 1);
    lambda2 = T / 2.0 + std::sqrt(T * T / 4.0 - D);
    lambda1 = T / 2.0 - std::sqrt(T * T / 4.0 - D);

    double c = std::abs(A.a(1, 0)), b = std::abs(A.a(0, 1));
    if (std::max(b, c) > EIGTOLERANCE)
    {
        if (b > c)
        {
            e1 = corecvs::Vector2dd(A.a(0, 1), lambda1 - A.a(0, 0)).normalised();
            e2 = corecvs::Vector2dd(A.a(0, 1), lambda2 - A.a(0, 0)).normalised();
        }
        else
        {
            e1 = corecvs::Vector2dd(lambda1 - A.a(1, 1), A.a(1, 0)).normalised();
            e2 = corecvs::Vector2dd(lambda2 - A.a(1, 1), A.a(1, 0)).normalised();
        }
    }
    else
    {
        e1 = corecvs::Vector2dd(1, 0);
        e2 = corecvs::Vector2dd(0, 1);
    }
}

void ChessBoardCornerDetector::adjustCornerOrientation()
{
    int iw = du.w, ih = du.h;
    for (auto& c: corners)
    {
        corecvs::Matrix A1(2, 2), A2(2, 2);

        int top = std::max(0.0, c.pos[1] - neighborhood);
        int bottom = std::min(ih - 1.0, c.pos[1] + neighborhood);
        int left = std::max(0.0, c.pos[0] - neighborhood);
        int right = std::min(iw - 1.0, c.pos[0] + neighborhood);

        for (int i = top; i <= bottom; ++i)
        {
            for (int j = left; j <= right; ++j)
            {
                corecvs::Vector2dd g(du.element(i, j), dv.element(i, j));
                if (g.l2Metric() < gradThreshold)
                    continue;
                g.normalise();
                if (std::abs(g & c.v1) < orientationInlierThreshold)
                {
                    for (int k = 0; k < 2; ++k)
                        for (int l = 0; l < 2; ++l)
                            A1.a(k, l) += (g[k] * g)[l];
                }
                if (std::abs(g & c.v2) < orientationInlierThreshold)
                {
                    for (int k = 0; k < 2; ++k)
                        for (int l = 0; l < 2; ++l)
                            A2.a(k, l) += (g[k] * g)[l];
                }
            }
        }

        corecvs::Vector2dd e1, e2, e;
        double l1, l2;
        eig22(A1, l1, e1, l2, e);
        eig22(A2, l1, e2, l2, e);
        c.v1 = (c.v1 & e1) > 0.0 ? e1 : -e1;
        c.v2 = (c.v2 & e2) > 0.0 ? e2 : -e2;
    }
}

bool ChessBoardCornerDetector::invertable22(corecvs::Matrix &A)
{
    assert(A.w == 2 && A.h == 2);
    const double DETTOLERANCE22 = 1e-9;
    return std::abs(A.a(0, 0) * A.a(1, 1) - A.a(1, 0) * A.a(0, 1)) > DETTOLERANCE22;
}

void ChessBoardCornerDetector::solve22(corecvs::Matrix &A, corecvs::Vector2dd &B, corecvs::Vector2dd &x)
{
    assert(A.w == 2 && A.h == 2);
    double a = A.a(0, 0), b = A.a(0, 1), c = A.a(1, 0), d = A.a(1, 1);
    double D = a * d - b * c;
    x[0] = ( d * B[0] - b * B[1]) / D;
    x[1] = (-c * B[0] + a * B[1]) / D;
}

void ChessBoardCornerDetector::adjustCornerPosition()
{
    int iw = du.w, ih = du.h;
    int idx = 0;
    for (auto& c: corners)
    {
        corecvs::Matrix G(2, 2);
        corecvs::Vector b(2);
        // FIXME: current corecvs::Vector implementation does not zero itself on init
        b.at(0) = 0.0; b.at(1) = 0.0;

        int top = std::max(0.0, c.pos[1] - neighborhood);
        int bottom = std::min(ih - 1.0, c.pos[1] + neighborhood);
        int left = std::max(0.0, c.pos[0] - neighborhood);
        int right = std::min(iw - 1.0, c.pos[0] + neighborhood);

        int cu = c.pos[0];
        int cv = c.pos[1];
        int inliers = 0;

        for (int i = top; i <= bottom; ++i)
        {
            for (int j = left; j <= right; ++j)
            {
                if (cu == j && cv == i)
                {
                    continue;
                }

                corecvs::Vector2dd g(du.element(i, j), dv.element(i, j));
                if (g.l2Metric() < gradThreshold)
                    continue;
                g.normalise();

                corecvs::Vector2dd p(j, i);
                corecvs::Vector2dd d = p - c.pos;

                double d1 = (d - (d & c.v1) * c.v1).l2Metric();
                double d2 = (d - (d & c.v2) * c.v2).l2Metric();

                if ((d1 < inlierDistanceThreshold && std::abs(g & c.v1) < orientationInlierThreshold) ||
                    (d2 < inlierDistanceThreshold && std::abs(g & c.v2) < orientationInlierThreshold))
                {
                    // TODO: outer product for vectors?!
                    corecvs::Matrix D(1, 2);
                    D.a(0, 0) = du.element(i, j);
                    D.a(0, 1) = dv.element(i, j);
                    auto H = D.t() * D;
                    corecvs::Vector DD(2);
                    DD.at(0) = j;
                    DD.at(1) = i;

                    inliers++;
                    G = G + H;
                    b = b +  H * DD;
                }

            }
        }

        if (!invertable22(G))
        {
            continue;
        }

        corecvs::Vector2dd x(0, 0), B(b.at(0), b.at(1));
        solve22(G, B, x);

        if ((x - c.pos).l2Metric() > updateThreshold)
        {
            continue;
        }
        c.pos = x;

        corners[idx++] = c;
    }
    corners.resize(idx);
}


void ChessBoardCornerDetector::computeScores()
{
    int idx = 0;
    for (auto& c: corners)
    {
        if (c.scoreCorner(img, w, cornerScores) < scoreThreshold)
            continue;

        // ok, here we also re-orient'em
        if (c.v2[0] < 0.0) c.v2 = -c.v2;
        if ((c.v1 & c.v2.rightNormal()) < 0.0) c.v1 = -c.v1;

        corners[idx++] = c;
    }
    corners.resize(idx);
}

void ChessBoardCornerDetector::detectCorners(DpImage &image, std::vector<OrientedCorner> &corners_)
{
    corners.clear();
    img = image;

    scaleImage();
    prepareDiff(du, true);
    prepareDiff(dv, false);
    prepareAngleWeight();

    computeCost();
    runNms();

    filterByOrientation();
    adjustCornerOrientation();
    adjustCornerPosition();

    for (int i = 0; i < nRounds; ++i)
    {
        adjustCornerOrientation();
        adjustCornerPosition();
    }

    computeScores();
    corners_ = corners;
}

ChessBoardCornerDetector::ChessBoardCornerDetector(ChessBoardCornerDetectorParams params) : ChessBoardCornerDetectorParams(params)
{
    prepareKernels();
}
