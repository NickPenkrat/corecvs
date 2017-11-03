#include "core/patterndetection/chessBoardCornerDetector.h"
#include "core/buffers/nonMaximalSuperssor.h"
#include "core/math/mathUtils.h"
#include "convolver/convolver.h"
#include "core/buffers/kernels/fastkernel/vectorTraits.h"
#include "core/buffers/kernels/fastkernel/fastKernel.h"
#include "core/buffers/kernels/arithmetic.h"
#include "core/math/matrix/matrix22.h"
#include "core/stats/calculationStats.h"

#include <cmath>
#include <algorithm>
#include <set>
#include <regex>

using corecvs::Vector2d;
using corecvs::Vector2dd;
using corecvs::Matrix22;
using corecvs::ConvolveKernel;
using corecvs::DummyAlgebra;
using corecvs::Statistics;
using corecvs::NonMaximalSuperssor;

using corecvs::FpKernel;
using corecvs::FpImage;
using corecvs::RGB24Buffer;
using corecvs::Convolver;
using corecvs::normalPDF;



template<class KernelType>
void CornerKernelSet::minifyKernel(KernelType &k)
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


double OrientedCorner::scoreCorner(DpImage &img, DpImage &weight, const std::vector<double> &radius, double bandwidth)
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
    double K_sum = -kw * kw;
    double K_sum_sq = kw * kw;

    double I_sum = 0.0, I_sum_sq = 0.0;
    int cx = r, cy = r;
    int ui = pos[0], vi = pos[1];
    Vector2dd c(r, r);

    for (int i = 0; i < kw; ++i)
    {
        for (int j = 0; j < kw; ++j)
        {
            Vector2dd p(j, i);
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
    double alpha = v1.argument();
    double psi   = v2.argument() - alpha;

    CornerKernelSet cks(r, alpha, psi);
    DpImage patch(w, w);
    int uc = pos.x();
    int vc = pos.y();

    int left   = std::max(0,         r - uc);
    int right  = std::min(w, img.w + r - uc);
    int top    = std::max(0,         r - vc);
    int bottom = std::min(w, img.h + r - vc);

    for (int i = 0; i < w; i++)
    {
        for (int j = 0; j < w; j++)
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
    return std::max(c.element(cks.K[0].y, cks.K[0].x), 0.0);
}



CornerKernelSet::CornerKernelSet(double r, double alpha, double psi, bool minify)
{
    int w = ((int)(r + 1.0)) * 2 + 1;

    for (int i = 0; i < KERNEL_LAST; i++ ) {
         K[i] = DpKernel(w, w);
        fK[i] = FpKernel(w, w);

    }

    int c = w / 2;

    computeKernels( K, r, alpha, psi, w, c);
    computeKernels(fK, r, alpha, psi, w, c);

    /*Or we could just copy...*/
    if (minify)
    {
        for (int i = 0; i < KERNEL_LAST; i++ ) {
            minifyKernel<DpKernel>( K[i]);
            minifyKernel<FpKernel>(fK[i]);
        }
    }
}



void CornerKernelSet::computeCost(DpImage &img, DpImage &c, bool parallelable, bool new_style)
{
    int w = img.w, h = img.h;
    c = DpImage(h, w);

    DpImage *pf[KERNEL_LAST];
    for (int i = 0; i < KERNEL_LAST; i++)
       pf[i] = 0;

    if (!new_style)
    {
        for (int i = 0; i < KERNEL_LAST; i++)
           pf[i] = img.doConvolve<DpImage>(&K[i], true, parallelable);
    }
    else
    {
        // TODO: I hope we'll change the design of the whole thing
        for (int i = 0; i < KERNEL_LAST; i++)
           pf[i] = new DpImage(img.h, img.w, false);

        Convolver conv;

        for (int i = 0; i < KERNEL_LAST; i++)
            conv.convolve(img, K[i], *pf[i]);
    }

    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            double fA = pf[KERNEL_A]->element(i, j),
                   fB = pf[KERNEL_B]->element(i, j),
                   fC = pf[KERNEL_C]->element(i, j),
                   fD = pf[KERNEL_D]->element(i, j);

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
    for (int i = 0; i < KERNEL_LAST; i++)
        delete_safe(pf[i]);
}

void CornerKernelSet::computeCost(FpImage &img, DpImage &c)
{
    int w = img.w, h = img.h;
    c = DpImage(h, w);

    FpImage *pf[KERNEL_LAST];
    for (int i = 0; i < KERNEL_LAST; i++)
    {
        pf[i] = new FpImage(img.h, img.w, false);
    }

    Convolver conv;
    for (int i = 0; i < KERNEL_LAST; i++)
        conv.convolve(img, fK[i], *pf[i]);

    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            float fA = pf[KERNEL_A]->element(i, j),
                  fB = pf[KERNEL_B]->element(i, j),
                  fC = pf[KERNEL_C]->element(i, j),
                  fD = pf[KERNEL_D]->element(i, j);

            float mu = 0.25f * (fA + fB + fC + fD);
            float a = std::min(fA - mu, fB - mu);
            float b = std::min(mu - fC, mu - fD);
            float r1 = std::min(a, b);
            a = std::min(fC - mu, fD - mu);
            b = std::min(mu - fA, mu - fB);
            float r2 = std::min(a, b);

            c.element(i, j) = std::max(r1, r2);
        }
    }

    // FIXME: we should definitely change convolution routine
    for (int i = 0; i < KERNEL_LAST; i++)
        delete_safe(pf[i]);
}

template<class KernelType>
void CornerKernelSet::computeKernels(KernelType K[], double r, double alpha, double psi, int w, int c, double threshold)
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

            for (int k = 0; k < KERNEL_LAST; k++)
                K[k].element(i,j) = 0.0;

            if (p1 > threshold && p2 < -threshold)
            {
                K[KERNEL_A].element(i, j) = normalPDF(lr, sigma);
            }
            if (p1 < -threshold && p2 > threshold)
            {
                K[KERNEL_B].element(i, j) = normalPDF(lr, sigma);
            }
            if (p1 < -threshold && p2 < -threshold)
            {
                K[KERNEL_C].element(i, j) = normalPDF(lr, sigma);
            }
            if (p1 > threshold && p2 > threshold)
            {
                K[KERNEL_D].element(i, j) = normalPDF(lr, sigma);
            }
        }
    }
#if 0
    std::cout << "[" << std::endl;
    for (int k = 0; k < KERNEL_LAST; k++)
    {
        for (int i = 0; i < w; ++i)
        {
            for (int j = 0; j < w; ++j)
            {
                std::cout << K[k].element(i, j) << " ";
            }
            std::cout << ";" << std::endl;
        }
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
void ChessBoardCornerDetector::scaleImage(double percLow/* = 0.05*/, double percHigh/* = 0.95*/)
{
    std::vector<double> values;
    values.reserve(img.h * img.w);

    for (int i = 0; i < img.h; ++i)
    {
        for (int j = 0; j < img.w; ++j)
        {
            values.push_back(img.element(i, j));
        }
    }

    std::sort(values.begin(), values.end());

    double pL = values[percLow  * values.size()];   // TODO: for Indoors: 0.002-0.998
    double pH = values[percHigh * values.size()];
    double dp = pH - pL;

    for (int i = 0; i < img.h; ++i)
    {
        for (int j = 0; j < img.w; ++j)
        {
            img.element(i, j) = std::max(std::min((img.element(i, j) - pL) / dp, 1.0), 0.0);
        }
    }
}

void ChessBoardCornerDetector::prepareKernels()
{
    kernels.clear();

    for (double r: patternRadius())
    {
        for (double psi: patternStartAngle())
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

    Statistics::enterContext(stats, "Cost->");

    FpImage *fImage = NULL;
    if (floatSpeedup())
    {
        Statistics::startInterval(stats);
            fImage = new FpImage(img.getSize());
            fImage->binaryOperationInPlace(img, [](const float &/*a*/, const double &b) { return b; });
        Statistics::endInterval(stats, "Float converter");
    }

    for (size_t i = 0; i < kernels.size(); i++)
    {
        Statistics::startInterval(stats);

        CornerKernelSet &cks = kernels[i];
        DpImage kc;
        if (floatSpeedup()) {
            cks.computeCost(*fImage, kc);
        } else {
            cks.computeCost(img, kc);
        }

        cost.binaryOperationInPlace(kc, [](const double &a, const double &b) { return std::max(a, b); });

        Statistics::endInterval(stats, "kernel" + std::to_string(i));
    }

    delete_safe(fImage);

    Statistics::leaveContext(stats);
#else
    for (int i = 0; i < kernels.size(); ++i)
    {
        cost.binaryOperationInPlace(kc[i], [](const double &a, const double &b) { return std::max(a, b); });
    }
#endif
}

void ChessBoardCornerDetector::runNms()
{
    typedef Vector2d<int> CoordType;
    std::vector<CoordType> cornerCandidates;
    NonMaximalSuperssor<DpImage> suppressor;

    suppressor.nonMaximumSupression(cost, nmsLocality(), nmsThreshold(), cornerCandidates, nmsLocality());
    for (CoordType& cc: cornerCandidates)
    {
        corners.emplace_back(Vector2dd(cc.x(), cc.y()));
    }
}


void ChessBoardCornerDetector::circularMeanShift(std::vector<double> &values, double bandwidth, std::vector<std::pair<int, double>> &modes)
{
    int N = (int)values.size();
    std::vector<double> smoothed(N);

    for (int i = 0; i < N; ++i)
    {
        double sum = 0.0;
        for (int j = (int)-std::ceil(3.0 * bandwidth); j <= (int)std::ceil(3.0 * bandwidth); ++j)
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

/**
 *   DpImage du, dv, w, phi, cost, img;
 *   std::vector<CornerKernelSet> kernels;
 **/
vector<std::string> ChessBoardCornerDetector::debugBuffers() const
{
    vector<std::string> result;
    result.push_back("du");
    result.push_back("dv");

    result.push_back("w");

    result.push_back("phi");
    result.push_back("cost");
    result.push_back("img");

    for (size_t i = 0; i < kernels.size(); i++)
    {
        for (int k = 0; k < CornerKernelSet::KERNEL_LAST; k++) {
            result.push_back(std::string("kernel_") + std::to_string(k) + "_"  +  std::to_string(i));
        }
    }
    return result;
}

RGB24Buffer *ChessBoardCornerDetector::getDebugBuffer(const std::string& name) const
{
    RGB24Buffer *result = NULL;
    if (name == "du") {
        result = new RGB24Buffer(du.h, du.w);
        result->drawDoubleBuffer(du);
    }
    if (name == "dv") {
        result = new RGB24Buffer(dv.h, dv.w);
        result->drawDoubleBuffer(dv);
    }
    if (name == "w") {
        result = new RGB24Buffer(w.h, w.w);
        result->drawDoubleBuffer(w);
    }
    if (name == "phi") {
        result = new RGB24Buffer(phi.h, phi.w);
        result->drawDoubleBuffer(phi);
    }
    if (name == "cost") {
        result = new RGB24Buffer(cost.h, cost.w);
        result->drawDoubleBuffer(cost);
    }
    if (name == "img") {
        result = new RGB24Buffer(img.h, img.w);
        result->drawDoubleBuffer(img);
    }

    std::regex regexp("^kernel_([0-9]*)_([0-9]*)$");
    std::smatch m;
    bool res = std::regex_search(name, m, regexp);
    if (res) {
        SYNC_PRINT(("ChessBoardCornerDetector::getDebugBuffer(): Parsed to %s %s %s", m[0].str().c_str(), m[1].str().c_str(), m[2].str().c_str()));

        size_t id   = std::stoi(m[2]);
        size_t knum = std::stoi(m[1]);

        if (id < kernels.size() && knum < CornerKernelSet::KERNEL_LAST) {
            result = new RGB24Buffer(kernels[id].K[knum].getSize());
            result->drawDoubleBuffer(kernels[id].K[knum]);
        }
    }
    else {
        SYNC_PRINT(("ChessBoardCornerDetector::getDebugBuffer(): no match for <%s>", name.c_str() ));
    }
    return result;
}

bool ChessBoardCornerDetector::edgeOrientationFromGradient(int top, int bottom, int left, int right, Vector2dd &v1, Vector2dd &v2)
{
    int nbins = histogramBins();
    if (nbins < 1)
    {
        std::cout << "ChessBoardCornerDetector:: bad histogramBins value:" << nbins << std::endl;
        return false;
    }

    std::vector<double> histogram(nbins);
    double bin_size = M_PI / nbins;

    for (int i = top; i <= bottom; ++i)
    {
        for (int j = left; j <= right; ++j)
        {
            double Phi = phi.element(i, j);
            double W   =   w.element(i, j);

            int bin = std::max(std::min((int)(Phi / bin_size), nbins - 1), 0);
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
        OrientedCorner& c = corners[i];

        int top    = (int)std::max(0.0     , c.pos[1] - neighborhood());
        int bottom = (int)std::min(ih - 1.0, c.pos[1] + neighborhood());
        int left   = (int)std::max(0.0     , c.pos[0] - neighborhood());
        int right  = (int)std::min(iw - 1.0, c.pos[0] + neighborhood());

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
    for (OrientedCorner& c: corners)
    {
        Matrix22 A1(0.0);
        Matrix22 A2(0.0);

        int top    = (int)std::max(0.0     , c.pos.y() - neighborhood());
        int bottom = (int)std::min(ih - 1.0, c.pos.y() + neighborhood());
        int left   = (int)std::max(0.0     , c.pos.x() - neighborhood());
        int right  = (int)std::min(iw - 1.0, c.pos.x() + neighborhood());

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

        int top    = (int)std::max(0.0     , c.pos.y() - neighborhood());
        int bottom = (int)std::min(ih - 1.0, c.pos.y() + neighborhood());
        int left   = (int)std::max(0.0     , c.pos.x() - neighborhood());
        int right  = (int)std::min(iw - 1.0, c.pos.x() + neighborhood());

        int cu = (int)c.pos[0];
        int cv = (int)c.pos[1];
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
        if (c.scoreCorner(img, w, cornerScores()) < scoreThreshold())
            continue;

        // ok, here we also re-orient'em
        if (c.v2.x() < 0.0)
            c.v2 = -c.v2;
        if ((c.v1 & c.v2.rightNormal()) < 0.0)
            c.v1 = -c.v1;

        corners[idx++] = c;
    }
    corners.resize(idx);
}

void ChessBoardCornerDetector::detectCorners(DpImage &image, std::vector<OrientedCorner> &corners_)
{
    Statistics::startInterval(stats);

    corners.clear();
    img = image;

    double lowPercentile  = normalizePercentile() / 100.0;
    double highPercentile = 1.0 - lowPercentile;

    scaleImage(lowPercentile, highPercentile);

    Statistics::resetInterval(stats, "Scaling");

    prepareDiff(du, true);
    Statistics::resetInterval(stats, "U direction Diff Preparation");

    prepareDiff(dv, false);
    Statistics::resetInterval(stats, "V direction Diff Preparation");

    prepareAngleWeight();
    Statistics::resetInterval(stats, "Angle weight Preparation");

    computeCost();
    Statistics::resetInterval(stats, "Cost");

    runNms();
    Statistics::resetInterval(stats, "NMS");

    filterByOrientation();
    adjustCornerOrientation();
    adjustCornerPosition();
    Statistics::resetInterval(stats, "Adjusting first round");

    for (int i = 0; i < nRounds(); ++i)
    {
        adjustCornerOrientation();
        adjustCornerPosition();
    }

    Statistics::resetInterval(stats, "Adjusting next rounds");

    computeScores();
    corners_ = corners;
}

ChessBoardCornerDetector::ChessBoardCornerDetector(ChessBoardCornerDetectorParams params)
    : ChessBoardCornerDetectorParams(params)
{
    prepareKernels();
}

