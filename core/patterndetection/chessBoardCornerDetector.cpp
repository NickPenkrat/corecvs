#include "chessBoardCornerDetector.h"

#include <cmath>
#include <algorithm>
#include <set>

#include "mathUtils.h"

#include "vectorTraits.h"
#include "fastKernel.h"
#include "arithmetic.h"
#include "matrix22.h"


using corecvs::Vector2dd;
using corecvs::Matrix22;


#ifdef USE_UNSAFE_CONVOLUTOR
#include <immintrin.h>

#endif

double OrientedCorner::scoreCorner(DpImage &img, DpImage &weight, std::vector<double> &radius, double bandwidth)
{
    int iw = img.w;
    int ih = img.h;
    score = 0.0;

    for (double r: radius)
    {
        if (!pos.isInRect(Vector2dd(r - 1, r - 1), Vector2dd(iw - r, ih - r)))
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
    cks.computeCost(patch, c, true, false);
    return std::max(c.element(cks.A.y, cks.A.x), 0.0);
}

CornerKernelSet::CornerKernelSet(double r, double alpha, double psi, bool minify)
{
    int w = ((int)(r + 1.0)) * 2 + 1;
    A = DpKernel(w, w);
    B = DpKernel(w, w);
    C = DpKernel(w, w);
    D = DpKernel(w, w);

    int c = w / 2;

    computeKernels(r, alpha, psi, w, c);
    if (minify)
    {
        MinifyKernel(A);
        MinifyKernel(B);
        MinifyKernel(C);
        MinifyKernel(D);
    }
}

void CornerKernelSet::MinifyKernel(DpKernel &k)
{
    int t = 0, l = 0, d = k.h - 1, r = k.w - 1;
    bool isNull = true;
    for (; t < k.h && isNull; ++t)
    {
        for (int i = 0; i < k.w; ++i)
        {
            if (std::abs(k.element(t, i)) > 0.0)
            {
                isNull = false;
                break;
            }
        }
    }
    t--;
    isNull = true;
    for (; d >= 0 && isNull; --d)
    {
        for (int i = 0; i < k.w; ++i)
        {
            if (std::abs(k.element(d, i)) > 0.0)
            {
                isNull = false;
                break;
            }
        }
    }
    d++;
    isNull = true;
    for (; l < k.w && isNull; ++l)
    {
        for (int i = 0; i < k.h; ++i)
        {
            if (std::abs(k.element(i, l)) > 0.0)
            {
                isNull = false;
                break;
            }
        }
    }
    l--;
    isNull = true;
    for (; r >= 0 && isNull; --r)
    {
        for (int i = 0; i < k.h; ++i)
        {
            if (std::abs(k.element(i, r)) > 0.0)
            {
                isNull = false;
                break;
            }
        }
    }
    r++;
    if (l > k.x) l = k.x;
    if (t > k.y) t = k.y;
    if (r <= k.x) r = k.x + 1;
    if (d <= k.y) d = k.y + 1;
    if (l == 0 && t == 0 && d == k.h - 1 && r == k.w - 1)
        return;
    CORE_ASSERT_TRUE_S(d < k.h && r < k.w);
    for (int i = 0; i <= d - t; ++i)
    {
        for (int j = 0; j <= r - l; ++j)
        {
            k.element(i, j) = k.element(i + t, j + l);
        }
    }
    k.x = k.x - l;
    k.y = k.y - t;
    k.h = d - t + 1;
    k.w = r - l + 1;
    CORE_ASSERT_TRUE_S(k.x >= 0 && k.x < k.w && k.y >= 0 && k.y < k.h);
}

void CornerKernelSet::computeCost(DpImage &img, DpImage &c, bool parallelable, bool new_style)
{
    int w = img.w, h = img.h;
    c = DpImage(h, w);


    DpImage *pfA, *pfB, *pfC, *pfD;
    if (!new_style)
    {
       pfA = img.doConvolve<DpImage>(&A, true, parallelable);
       pfB = img.doConvolve<DpImage>(&B, true, parallelable);
       pfC = img.doConvolve<DpImage>(&C, true, parallelable);
       pfD = img.doConvolve<DpImage>(&D, true, parallelable);
    }
    else
    {
        // TODO: I hope we'll change the design of the whole thing
        pfA = new DpImage(img.h, img.w, false);
        pfB = new DpImage(img.h, img.w, false);
        pfC = new DpImage(img.h, img.w, false);
        pfD = new DpImage(img.h, img.w, false);
#ifndef USE_UNSAFE_CONVOLUTOR
        corecvs::ConvolveKernel<corecvs::DummyAlgebra> convA(&A, A.y, A.x);
        corecvs::ConvolveKernel<corecvs::DummyAlgebra> convB(&B, B.y, B.x);
        corecvs::ConvolveKernel<corecvs::DummyAlgebra> convC(&C, C.y, C.x);
        corecvs::ConvolveKernel<corecvs::DummyAlgebra> convD(&D, D.y, D.x);

        DpImage *in = &img;
        corecvs::BufferProcessor<DpImage, DpImage, corecvs::ConvolveKernel, corecvs::AlgebraDouble> proScalar;
        proScalar.process(&in, &pfA, convA);
        proScalar.process(&in, &pfB, convB);
        proScalar.process(&in, &pfC, convC);
        proScalar.process(&in, &pfD, convD);
#else
        unsafeConvolutor(img, A, *pfA);
        unsafeConvolutor(img, B, *pfB);
        unsafeConvolutor(img, C, *pfC);
        unsafeConvolutor(img, D, *pfD);
#endif
    }

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
    double sigma = r / 2.0;
    Vector2dd v1 = Vector2dd::FromPolar(alpha      ).rightNormal();
    Vector2dd v2 = Vector2dd::FromPolar(alpha + psi).rightNormal();
    Vector2dd cc(c, c);

    //double sum = 0.0;
    for (int i = 0; i < w; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            Vector2dd p(i, j);
            Vector2dd v = p - cc;
            // Now we check if it is inside => cross product changes sign
            double p1 = v & v1;
            double p2 = v & v2;
            double lr = !v;


            A.element(i, j) = B.element(i, j) = C.element(i, j) = D.element(i, j) = 0.0;
            if (p1 > threshold && p2 < -threshold)
            {
                A.element(i, j) = normalPDF(lr, sigma);
            }
            if (p1 < -threshold && p2 > threshold)
            {
                B.element(i, j) = normalPDF(lr, sigma);
            }
            if (p1 < -threshold && p2 < -threshold)
            {
                C.element(i, j) = normalPDF(lr, sigma);
            }
            if (p1 > threshold && p2 > threshold)
            {
                D.element(i, j) = normalPDF(lr, sigma);
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

    for (auto& r: patternRadius())
    {
        for (auto& psi: patternStartAngle())
        {
            kernels.emplace_back(r, psi, sectorSize(), true);
        }
    }
}

void ChessBoardCornerDetector::computeCost()
{
    int w = img.w, h = img.h;
    cost = DpImage(h, w);
#if 0
    std::vector<DpImage> kc(kernels.size());
    corecvs::parallelable_for(0, (int)kernels.size(),
            [&](const corecvs::BlockedRange<int> &r)
            {
                for (int i = r.begin(); i < r.end(); ++i)
                {
                    kernels[i].computeCost(img, kc[i]);
                }
            });
#endif
#if 10
    for (auto& cks: kernels)
    {
        DpImage kc;
        cks.computeCost(img, kc);

        cost.binaryOperationInPlace(kc, [](const double &a, const double &b) { return std::max(a, b); });
    }
#else
    for (int i = 0; i < kernels.size(); ++i)
    {
        cost.binaryOperationInPlace(kc[i], [](const double &a, const double &b) { return std::max(a, b); });
    }
#endif
}

void ChessBoardCornerDetector::runNms()
{
    std::vector<std::pair<int, int>> cornerCandidates;
    cost.nonMaximumSupression(nmsLocality(), 0.025, cornerCandidates, nmsLocality());
    for (auto& cc: cornerCandidates)
    {
        corners.emplace_back(corecvs::Vector2dd(cc.first, cc.second));
    }
}


void ChessBoardCornerDetector::circularMeanShift(std::vector<double> &values, double bandwidth, std::vector<std::pair<int, double>> &modes)
{
    int N = (int)values.size();
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
            int left  = (i + N - 1) % N;
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

void ChessBoardCornerDetector::setStatistics(corecvs::Statistics *stats)
{
    this->stats = stats;
}

corecvs::Statistics *ChessBoardCornerDetector::getStatistics()
{
    return stats;
}

bool ChessBoardCornerDetector::edgeOrientationFromGradient(int top, int bottom, int left, int right, corecvs::Vector2dd &v1, corecvs::Vector2dd &v2)
{
    std::vector<double> histogram(histogramBins());
    double bin_size = M_PI / histogramBins();

    for (int i = top; i <= bottom; ++i)
    {
        for (int j = left; j <= right; ++j)
        {
            double Phi = phi.element(i, j);
            double W   =   w.element(i, j);

            int bin = std::max(std::min((int)(Phi / bin_size), histogramBins() - 1), 0);
            histogram[bin] += W;
        }
    }

    typedef std::pair<int, double> PairID;
    std::vector<PairID > modes;
    circularMeanShift(histogram, meanshiftBandwidth(), modes);

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
    if (angle < minAngle())
    {
        return false;
    }

    v1 = Vector2dd::FromPolar(phi1);
    v2 = Vector2dd::FromPolar(phi2);
    return true;
}

void ChessBoardCornerDetector::filterByOrientation()
{
    int idx = 0, N = (int)corners.size(), iw = w.w, ih = w.h;
    for (int i = 0; i < N; ++i)
    {
        auto& c = corners[i];

        int top    = std::max(     0.0, c.pos[1] - neighborhood());
        int bottom = std::min(ih - 1.0, c.pos[1] + neighborhood());
        int left   = std::max(     0.0, c.pos[0] - neighborhood());
        int right  = std::min(iw - 1.0, c.pos[0] + neighborhood());

        if (edgeOrientationFromGradient(top, bottom, left, right, c.v1, c.v2))
        {
            corners[idx++] = c;
        }
    }
    corners.resize(idx);
}

void ChessBoardCornerDetector::adjustCornerOrientation()
{
    int iw = du.w, ih = du.h;
    for (auto& c: corners)
    {
        Matrix22 A1(0.0);
        Matrix22 A2(0.0);

        int top    = std::max(     0.0, c.pos.y() - neighborhood());
        int bottom = std::min(ih - 1.0, c.pos.y() + neighborhood());
        int left   = std::max(     0.0, c.pos.x() - neighborhood());
        int right  = std::min(iw - 1.0, c.pos.x() + neighborhood());

        for (int i = top; i <= bottom; ++i)
        {
            for (int j = left; j <= right; ++j)
            {
                corecvs::Vector2dd g(du.element(i, j), dv.element(i, j));
                if (g.l2Metric() < gradThreshold())
                    continue;
                g.normalise();
                if (std::abs(g & c.v1) < orientationInlierThreshold())
                {
                    for (int k = 0; k < 2; ++k)
                        for (int l = 0; l < 2; ++l)
                            A1.a(k, l) += (g[k] * g)[l];
                }
                if (std::abs(g & c.v2) < orientationInlierThreshold())
                {
                    for (int k = 0; k < 2; ++k)
                        for (int l = 0; l < 2; ++l)
                            A2.a(k, l) += (g[k] * g)[l];
                }
            }
        }

        Vector2dd e1, e2, e;
        double l1, l2;
        Matrix22::eigen(A1, l1, e1, l2, e);
        Matrix22::eigen(A2, l1, e2, l2, e);
        c.v1 = (c.v1 & e1) > 0.0 ? e1 : -e1;
        c.v2 = (c.v2 & e2) > 0.0 ? e2 : -e2;
    }
}

void ChessBoardCornerDetector::adjustCornerPosition()
{
    int iw = du.w, ih = du.h;
    int idx = 0;
    for (auto& c: corners)
    {
        Matrix22  G(0.0);
        Vector2dd b(0.0, 0.0);
        // FIXME: current corecvs::Vector implementation does not zero itself on init

        int top    = std::max(     0.0, c.pos.y() - neighborhood());
        int bottom = std::min(ih - 1.0, c.pos.y() + neighborhood());
        int left   = std::max(     0.0, c.pos.x() - neighborhood());
        int right  = std::min(iw - 1.0, c.pos.x() + neighborhood());

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

                Vector2dd g(du.element(i, j), dv.element(i, j));
                if (g.l2Metric() < gradThreshold())
                    continue;
                g.normalise();

                Vector2dd p(j, i);
                Vector2dd d = p - c.pos;

                double d1 = (d - (d & c.v1) * c.v1).l2Metric();
                double d2 = (d - (d & c.v2) * c.v2).l2Metric();

                if ((d1 < inlierDistanceThreshold() && std::abs(g & c.v1) < orientationInlierThreshold()) ||
                    (d2 < inlierDistanceThreshold() && std::abs(g & c.v2) < orientationInlierThreshold()))
                {
                    // TODO: outer product for vectors?!
                    Vector2dd D(du.element(i, j), dv.element(i, j));
                    Matrix22 H = Matrix22::VectorByVector(D, D);
                    Vector2dd DD(j, i);

                    inliers++;
                    G = G + H;
                    b = b + H * DD;
                }

            }
        }

        if (!G.isInvertable())
        {
            continue;
        }

        Vector2dd x = Matrix22::solve(G, b);

        if ((x - c.pos).l2Metric() > updateThreshold())
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
        if (c.scoreCorner(img, w, cornerScores) < scoreThreshold())
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
    if (stats != NULL) stats->startInterval();

    corners.clear();
    img = image;   
    scaleImage();

    if (stats != NULL) stats->resetInterval("Scaling");

    prepareDiff(du, true);
    prepareDiff(dv, false);
    prepareAngleWeight();

    if (stats != NULL) stats->resetInterval("Diff Preparation");

    computeCost();
    runNms();

    if (stats != NULL) stats->resetInterval("Cost and NMS");

    filterByOrientation();
    adjustCornerOrientation();
    adjustCornerPosition();

    if (stats != NULL) stats->resetInterval("Adjusting first round");

    for (int i = 0; i < nRounds(); ++i)
    {
        adjustCornerOrientation();
        adjustCornerPosition();
    }

    if (stats != NULL) stats->resetInterval("Adjusting next rounds");

    computeScores();
    corners_ = corners;
}

ChessBoardCornerDetector::ChessBoardCornerDetector(ChessBoardCornerDetectorParams params) :
    ChessBoardCornerDetectorParams(params),
    stats(NULL)
{
    prepareKernels();
}

