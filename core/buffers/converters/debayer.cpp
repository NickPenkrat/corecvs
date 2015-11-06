#include "debayer.h"
#include <limits>

Debayer::Debayer(G12Buffer *bayer, int depth, MetaData *metadata)
    : mBayer(bayer)
    , mDepth(depth)
    , mMetadata(metadata)
{}

Debayer::~Debayer()
{
    deletearr_safe(mCurve);
}

// I don't understand what this actually does...
// TODO: maybe just replace it with simple 3-colour loop?
//
int FC(int row, int col, int filters)
{
    return (filters >> (((row << 1 & 14) | (col & 1)) << 1) & 3);
}

uint16_t Debayer::clip(int32_t x, int depth)
{
    const uint16_t maximum = (1 << depth) - 1;
    if (x < 0)
        return 0;
    if (x > maximum || x >= (1 << 16))
        return maximum;
    return (uint16_t)x;
}

RGB48Buffer* Debayer::nearest()
{
    // RGGB
    // TODO: write this to metadata
    int bpos = 0;
    uint16_t red = 0, green = 0, blue = 0;

    // swapCols inverts least significant bit for cols when set so RG/GB becomes GR/BG, etc.
    // swapRows does the same for rows
    int swapCols = bpos & 1;
    int swapRows = (bpos & 2) >> 1;
    RGB48Buffer *result = new RGB48Buffer(mBayer->h, mBayer->w, false);

    for (int i = 0; i < mBayer->h; i += 2)
        for (int j = 0; j < mBayer->w; j += 2)
        {
            for (int k = 0; k < 2; k++)
                for (int l = 0; l < 2; l++)
                {
                    int pxshift = (l ^ (l & 1));
                    // i don't know how i came to this
                    red   = mBayer->element(i + swapRows, (j + pxshift) ^ swapCols);
                    // green1 for even rows, green2 for odd
                    green = mBayer->element(i + !(k ^ swapRows), j + pxshift + !(k ^ !swapCols));
                    blue  = mBayer->element(i + !swapRows, (j + pxshift) ^ !swapCols);

                    result->setElement(i + k, j + l, RGBColor48(
                        mCurve[clip((int64_t)((red   - mBlack) * mScaleMul[0]), mDepth)],
                        mCurve[clip((int64_t)((green - mBlack) * mScaleMul[1]), mDepth)],
                        mCurve[clip((int64_t)((blue  - mBlack) * mScaleMul[2]), mDepth)]
                        ));
                }
        }

    return result;
}

RGB48Buffer* Debayer::linear()
{
    // RGGB
    // TODO: write this to metadata
    int bpos = 0;
    uint16_t red = 0, green = 0, blue = 0;

    int swapCols = bpos & 1;
    int swapRows = (bpos & 2) >> 1;
    RGB48Buffer *result = new RGB48Buffer(mBayer->h, mBayer->w, false);

    for (int i = 0; i < mBayer->h; i += 2)
        for (int j = 0; j < mBayer->w; j += 2)
        {
            for (int k = 0; k < 2; k++)
                for (int l = 0; l < 2; l++)
                {
                    int color = (l ^ swapCols) % 2 + ((k ^ swapRows) % 2) * 2;

                    switch (color)
                    {
                    case 0: // red
                        red = mBayer->element(i + k, j + l);
                        green = clampedBayerSum({ Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1), Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k) });
                        blue = clampedBayerSum({ Vector2d32(j + l - 1, i + k - 1), Vector2d32(j + l + 1, i + k - 1), Vector2d32(j + l - 1, i + k + 1), Vector2d32(j + l + 1, i + k + 1) });
                        break;
                    case 1: // green1
                        red = clampedBayerSum({ Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k) });
                        green = mBayer->element(i + k, j + l);
                        blue = clampedBayerSum({ Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1) });
                        break;
                    case 2: // green2
                        red = clampedBayerSum({ Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1) });
                        green = mBayer->element(i + k, j + l);
                        blue = clampedBayerSum({ Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k) });
                        break;
                    case 3: // blue
                        red = clampedBayerSum({ Vector2d32(j + l - 1, i + k - 1), Vector2d32(j + l + 1, i + k - 1), Vector2d32(j + l - 1, i + k + 1), Vector2d32(j + l + 1, i + k + 1) });
                        green = clampedBayerSum({ Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1), Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k) });
                        blue = mBayer->element(i + k, j + l);
                        break;
                    }

                    result->element(i + k, j + l) = {
                        mCurve[clip((int64_t)((red - mBlack)*mScaleMul[0]), mDepth)],
                        mCurve[clip((int64_t)((green - mBlack)*mScaleMul[1]), mDepth)],
                        mCurve[clip((int64_t)((blue - mBlack)*mScaleMul[2]), mDepth)]
                    };
                }
        }

    return result;
}

RGB48Buffer* Debayer::improved()
{
    // RGGB
    // TODO: write this to metadata
    int bpos = 0;
    uint16_t red = 0, green = 0, blue = 0;

    int swapCols = bpos & 1;
    int swapRows = (bpos & 2) >> 1;
    RGB48Buffer *result = new RGB48Buffer(mBayer->h, mBayer->w, false);
    //AbstractContiniousBuffer<double>* ratios = new AbstractContiniousBuffer<double>;

    for (int i = 0; i < mBayer->h; i += 2)
        for (int j = 0; j < mBayer->w; j += 2)
        {
            for (int k = 0; k < 2; k++)
                for (int l = 0; l < 2; l++)
                {
                    int color = (l ^ swapCols) % 2 + ((k ^ swapRows) % 2) * 2;
                    int deltar = 0, deltag = 0, deltab = 0, green_interp = 0, absh = 0, absv = 0;
                    double alpha = 0.3, beta = 0.625, gamma = 0.75;
                    switch (color)
                    {
                        // TODO: make the following code more readable
                    case 0: // red
                        deltar = mBayer->element(i + k, j + l) - clampedBayerSum({ Vector2d32(j + l, i + k - 2), Vector2d32(j + l, i + k + 2), Vector2d32(j + l - 2, i + k), Vector2d32(j + l + 2, i + k) });
                        absv = abs(clampedBayerSum(Vector2d32(j + l, i + k - 1)) - clampedBayerSum(Vector2d32(j + l, i + k + 1)));
                        absh = abs(clampedBayerSum(Vector2d32(j + l + 1, i + k)) - clampedBayerSum(Vector2d32(j + l - 1, i + k)));
                        //green_interp = clampedBayerSum(Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1), Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k));
                        green_interp = absv > absh ? clampedBayerSum({ Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k) }) :
                            clampedBayerSum({ Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1) });
                        red = mBayer->element(i + k, j + l);
                        green = clip(green_interp + alpha*deltar, mDepth);
                        blue = clip(clampedBayerSum({ Vector2d32(j + l - 1, i + k - 1), Vector2d32(j + l + 1, i + k - 1), Vector2d32(j + l - 1, i + k + 1), Vector2d32(j + l + 1, i + k + 1) }) + gamma*deltar, mDepth);
                        break;
                    case 1: // green1
                        deltag = mBayer->element(i + k, j + l) - clampedBayerSum({ Vector2d32(j + l - 1, i + k - 1), Vector2d32(j + l - 1, i + k + 1), Vector2d32(j + l + 1, i + k - 1), Vector2d32(j + l + 1, i + k + 1) });
                        red = clip(clampedBayerSum({ Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k) }) + beta*deltag, mDepth);
                        green = mBayer->element(i + k, j + l);
                        blue = clip(clampedBayerSum({ Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1) }) + beta*deltag, mDepth);
                        break;
                    case 2: // green2
                        deltag = mBayer->element(i + k, j + l) - clampedBayerSum({ Vector2d32(j + l - 1, i + k - 1), Vector2d32(j + l - 1, i + k + 1), Vector2d32(j + l + 1, i + k - 1), Vector2d32(j + l + 1, i + k + 1) });
                        red = clip(clampedBayerSum({ Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1) }) + beta*deltag, mDepth);
                        green = mBayer->element(i + k, j + l);
                        blue = clip(clampedBayerSum({ Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k) }) + beta*deltag, mDepth);;
                        break;
                    case 3: // blue
                        deltab = mBayer->element(i + k, j + l) - clampedBayerSum({ Vector2d32(j + l, i + k - 2), Vector2d32(j + l, i + k + 2), Vector2d32(j + l - 2, i + k), Vector2d32(j + l + 2, i + k) });
                        absv = abs(clampedBayerSum(Vector2d32(j + l, i + k - 1)) - clampedBayerSum(Vector2d32(j + l, i + k + 1)));
                        absh = abs(clampedBayerSum(Vector2d32(j + l + 1, i + k)) - clampedBayerSum(Vector2d32(j + l - 1, i + k)));
                        //green_interp = clampedBayerSum(Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1), Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k));
                        green_interp = absv > absh ? clampedBayerSum({ Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k) }) :
                            clampedBayerSum({ Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1) });
                        red = clip(clampedBayerSum({ Vector2d32(j + l - 1, i + k - 1), Vector2d32(j + l + 1, i + k - 1), Vector2d32(j + l - 1, i + k + 1), Vector2d32(j + l + 1, i + k + 1) }) + gamma*deltab, mDepth);
                        green = clip(green_interp + alpha*deltab, mDepth);
                        blue = mBayer->element(i + k, j + l);
                        break;
                    }

                    result->element(i + k, j + l) = {
                        mCurve[clip((int32_t)((red   - mBlack) * mScaleMul[0]), mDepth)],
                        mCurve[clip((int32_t)((green - mBlack) * mScaleMul[1]), mDepth)],
                        mCurve[clip((int32_t)((blue  - mBlack) * mScaleMul[2]), mDepth)]
                    };

                }
        }

    return result;
}

void Debayer::scaleCoeffs()
{
    for (int i = 0; i < 3; i++)
        mScaleMul[i] = 1;

    // check if metadata available
    if (mMetadata == nullptr || mMetadata->empty())
        return;

    // alias for ease of use
    MetaData &metadata = *mMetadata;

    // check if metadata valid
    if (metadata["cam_mul"].empty() || metadata["cam_mul"][0] == 0 || metadata["cam_mul"][2] == 0)
    {
        // if white balance is not available, do only scaling
        // white should be calculated before calling scaleCoeffs()
        if (!metadata["white"].empty() && metadata["white"][0] != 0)
        {
            double factor = ((1 << mDepth) - 1) / metadata["white"][0];
            for (int i = 0; i < 3; i++) {
                mScaleMul[i] = factor;
            }
            return;
        }

        // TODO: maybe apply the gray world hypothesis instead of doing nothing
        return;
    }

    unsigned c;
    double dmin;

    // if cam_mul coefficients are available directly, use them in future
    // if not, use pre_mul when available
    if (metadata["cam_mul"][0] != -1)
        for (int i = 0; i < 4; i++)
            metadata["pre_mul"][i] = metadata["cam_mul"][i];

    // if green scale coefficient is not available, set it to 1
    if (metadata["pre_mul"][1] == 0)
        metadata["pre_mul"][1] = 1;

    // black must be subtracted from the image, so we adjust maximum white level here
    metadata["white"][0] -= metadata["black"][0];

    // normalize pre_mul
    for (dmin = std::numeric_limits<double>::max(), c = 0; c < 4; c++)
    {
        if (dmin > metadata["pre_mul"][c])
            dmin = metadata["pre_mul"][c];
    }

    // scale colors to the desired bit-depth
    double factor = ((1 << mDepth) - 1) / metadata["white"][0];
    for (int c = 0; c < 3; c++)
        mScaleMul[c] = (metadata["pre_mul"][c] /= dmin) * factor;

    // black frame adjustment, not currently used as we don't have the black frame in metadata (use black level instead)
    // the black level is usually almost equal for all channels with difference less than 1 bit in 12-bit case
    // TODO: perform tests and prove the previous statement right or wrong (and maybe start using 3-channel black level)
    /*
    if (metadata->filters > 1000 && (metadata->cblack[4] + 1) / 2 == 1 && (metadata->cblack[5] + 1) / 2 == 1)
    {
        for (int c = 0; c < 4; c++)
        metadata->cblack[FC(c / 2, c % 2, metadata->filters)] +=
        metadata->cblack[6 + c / 2 % cblack[4] * cblack[5] + c % 2 % cblack[5]];
        metadata->cblack[4] = metadata->cblack[5] = 0;
    }*/
}

uint16_t* Debayer::gammaCurve(int imax)
{
    // this code is taken from LibRaw
    // TODO: rewrite?

    // initialize curve as a straight line (no correction)
    uint16_t *curve = new uint16_t[0x10000];
    for (int i = 0; i < 0x10000; i++)
        curve[i] = i;

    if (mMetadata == nullptr)
        return curve;
    // alias
    MetaData &metadata = *mMetadata;

    // if no gamm coefficients are present (or valid), return no transform
    if (metadata["gamm"].empty() || !(metadata["gamm"][0] || metadata["gamm"][1] || metadata["gamm"][2] || metadata["gamm"][3] || metadata["gamm"][4]))
        return curve;

    int i;
    double g[6], bnd[2] = { 0, 0 }, r;

    g[0] = metadata["gamm"][0];
    g[1] = metadata["gamm"][1];
    g[2] = g[3] = g[4] = 0;
    bnd[g[1] >= 1] = 1;
    if (g[1] && (g[1] - 1)*(g[0] - 1) <= 0)
    {
        for (i = 0; i < 48; i++)
        {
            g[2] = (bnd[0] + bnd[1]) / 2;
            if (g[0]) bnd[(pow(g[2] / g[1], -g[0]) - 1) / g[0] - 1 / g[2] > -1] = g[2];
            else	bnd[g[2] / exp(1 - 1 / g[2]) < g[1]] = g[2];
        }
        g[3] = g[2] / g[1];
        if (g[0]) g[4] = g[2] * (1 / g[0] - 1);
    }
    if (g[0]) g[5] = 1 / (g[1] * (g[3] * g[3]) / 2 - g[4] * (1 - g[3]) +
        (1 - pow(g[3], 1 + g[0]))*(1 + g[4]) / (1 + g[0])) - 1;
    else      g[5] = 1 / (g[1] * (g[3] * g[3]) / 2 + 1
        - g[2] - g[3] - g[2] * g[3] * (log(g[3]) - 1)) - 1;

    for (i = 0; i < 0x10000; i++)
    {
        curve[i] = (1 << mDepth) - 1;
        if ((r = (double)i / imax) < 1)
            curve[i] = (1 << mDepth) * (r < g[3] ? r*g[1] : (g[0] ? pow(r, g[0])*(1 + g[4]) - g[4] : log(r)*g[2] + 1));
    }
    return curve;
}

RGB48Buffer* Debayer::toRGB48(Quality quality)
{
    preprocess();

    RGB48Buffer *result = nullptr;
    switch (quality)
    {
    case Nearest:
        result = nearest();
        break;

    default:
    case Bilinear:
        result = linear();
        break;
    case Improved:
        result = improved();
        break;
    }

    return result;
}

void Debayer::preprocess(bool overwrite)
{
    // recalculate white balance coefficients
    scaleCoeffs();

    int t_white = 0;

    if (mMetadata != nullptr && (overwrite || mCurve == nullptr))
    {
        // alias for ease of use
        MetaData &metadata = *this->mMetadata;
        int m_bits = this->mMetadata != nullptr &&  metadata["bits"][0] ? metadata["bits"][0] : mDepth;
        int shift = m_bits - mDepth;

        mBlack = !metadata["black"].empty() && metadata["black"][0] ? metadata["black"][0] : 0;
        int m_white = !metadata["white"].empty() && metadata["white"][0] ? metadata["white"][0] : (1 << mDepth) - 1;
        int m_twhite = !metadata["t_white"].empty() && metadata["t_white"][0] ? (int)metadata["t_white"][0] : m_white;
        t_white = (shift < 0 ? m_twhite << -shift : m_twhite >> shift);
    }

    // (re)calculate gamma correction
    if (overwrite) {
        deletearr_safe(mCurve);
    }
    if (overwrite || mCurve == nullptr) {
        mCurve = gammaCurve(t_white);
    }
}

int32_t Debayer::clampedBayerSum(Vector2d32 coords)
{
    return clampedBayerSum(vector<Vector2d32>{coords});
}

int32_t Debayer::clampedBayerSum(vector<Vector2d32> coords)
{
    int32_t result = 0;
    uint16_t div = 0;

    for (vector<Vector2d32>::iterator it = coords.begin(); it != coords.end(); it++)
    {
        Vector2d32 v = (*it);
        if (v.x() >= 0 && v.y() >= 0 && v.x() < mBayer->w && v.y() < mBayer->h)
        {
            div++;
            result += mBayer->element(v);
        }
    }

    return div == 0 ? 0 : result / div;
}
