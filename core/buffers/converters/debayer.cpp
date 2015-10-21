#include "debayer.h"

Debayer::Debayer(G12Buffer *bayer, int depth, MetaData *metadata)
{
    this->mBayer = bayer;
    this->mMetadata = metadata;
    this->mDepth = depth;
}

// i have yet to understand what this actually does... 
// TODO: maybe just replace it with simple 3-colour loop?
int FC(int row, int col, int filters) { return (filters >> (((row << 1 & 14) | (col & 1)) << 1) & 3); }

uint16_t Debayer::clip(int64_t x, int depth)
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
    uint16_t red = 0, green = 0, blue = 0;

    RGB48Buffer *result = new RGB48Buffer(mBayer->h, mBayer->w, false);

    for (int i = 0; i < mBayer->h; i++)
        for (int j = 0; j < mBayer->w; j++)
        {
            bool is_red = !(j & 1) && !(i & 1);
            bool is_blue = (j & 1) && (i & 1);
            bool is_green1 = !is_red && !is_blue && !(i & 1);
            bool is_green2 = !is_red && !is_blue && (i & 1);

            if (is_red)
            {
                red = mBayer->element(i, j);
                green = mBayer->element(i, j + 1);
                blue = mBayer->element(i + 1, j + 1);
            }
            if (is_green1)
            {
                red = mBayer->element(i, j - 1);
                green = mBayer->element(i, j);
                blue = mBayer->element(i + 1, j);
            }
            if (is_green2)
            {
                red = mBayer->element(i - 1, j);
                green = mBayer->element(i, j);
                blue = mBayer->element(i, j + 1);
            }
            if (is_blue)
            {
                red = mBayer->element(i - 1, j - 1);
                green = mBayer->element(i, j - 1);
                blue = mBayer->element(i, j);
            }
            result->setElement(i, j, RGBColor48(
                mCurve[clip((int64_t)((red - mBlack)*mScaleMul[0]), mDepth)],
                mCurve[clip((int64_t)((green - mBlack)*mScaleMul[1]), mDepth)],
                mCurve[clip((int64_t)((blue - mBlack)*mScaleMul[2]), mDepth)]
                ));
        }

    return result;
}

RGB48Buffer* Debayer::linear()
{
    uint16_t red = 0, green = 0, blue = 0;

    RGB48Buffer *result = new RGB48Buffer(mBayer->h, mBayer->w, false);

    for (int i = 0; i < mBayer->h; i++)
        for (int j = 0; j < mBayer->w; j++)
        {
            bool is_red = !(j & 1) && !(i & 1);
            bool is_blue = (j & 1) && (i & 1);
            bool is_green1 = !is_red && !is_blue && !(i & 1);
            bool is_green2 = !is_red && !is_blue && (i & 1);

            if (is_red)
            {
                red = mBayer->element(i, j);

                if (!(i || j))
                {
                    green = (mBayer->element(i, j + 1) + mBayer->element(i + 1, j)) / 2;
                    blue = mBayer->element(i + 1, j + 1);
                }
                else if (i == mBayer->h - 1 && j == mBayer->w - 1)
                {
                    green = (mBayer->element(i, j - 1) + mBayer->element(i - 1, j)) / 2;
                    blue = mBayer->element(i - 1, j - 1);
                }
                else if (!i)
                {
                    green = (mBayer->element(i, j + 1) + mBayer->element(i + 1, j) + mBayer->element(i, j - 1)) / 3;
                    blue = (mBayer->element(i + 1, j + 1) + mBayer->element(i + 1, j - 1)) / 2;
                }
                else if (!j)
                {
                    green = (mBayer->element(i, j + 1) + mBayer->element(i + 1, j) + mBayer->element(i - 1, j)) / 3;
                    blue = (mBayer->element(i + 1, j + 1) + mBayer->element(i - 1, j + 1)) / 2;
                }
                else if (i == mBayer->h - 1)
                {
                    green = (mBayer->element(i, j - 1) + mBayer->element(i - 1, j) + mBayer->element(i, j + 1)) / 3;
                    blue = (mBayer->element(i - 1, j + 1) + mBayer->element(i - 1, j - 1)) / 2;
                }
                else if (j == mBayer->w - 1)
                {
                    green = (mBayer->element(i, j - 1) + mBayer->element(i + 1, j) + mBayer->element(i - 1, j)) / 3;
                    blue = (mBayer->element(i + 1, j - 1) + mBayer->element(i - 1, j - 1)) / 2;
                }
                else
                {
                    green = (mBayer->element(i, j + 1) + mBayer->element(i, j - 1) + mBayer->element(i + 1, j) + mBayer->element(i - 1, j)) / 4;
                    blue = (mBayer->element(i + 1, j + 1) + mBayer->element(i - 1, j + 1) + mBayer->element(i + 1, j - 1) + mBayer->element(i - 1, j - 1)) / 4;
                }

            }
            if (is_green1)
            {
                if (j != mBayer->w - 1)
                    red = (mBayer->element(i, j - 1) + mBayer->element(i, j + 1)) / 2;
                else
                    red = mBayer->element(i, j - 1);

                green = mBayer->element(i, j);

                if (i > 0)
                    blue = (mBayer->element(i + 1, j) + mBayer->element(i - 1, j)) / 2;
                else
                    blue = mBayer->element(i + 1, j);
            }
            if (is_green2)
            {
                if (i != mBayer->h - 1)
                    red = (mBayer->element(i + 1, j) + mBayer->element(i - 1, j)) / 2;
                else
                    red = mBayer->element(i - 1, j);

                green = mBayer->element(i, j);

                if (j > 0)
                    blue = (mBayer->element(i, j - 1) + mBayer->element(i, j + 1)) / 2;
                else
                    blue = mBayer->element(i, j - 1);

            }
            if (is_blue)
            {
                if (!(i || j))
                {
                    green = (mBayer->element(i, j + 1) + mBayer->element(i + 1, j)) / 2;
                    red = mBayer->element(i + 1, j + 1);
                }
                else if (i == mBayer->h - 1 && j == mBayer->w - 1)
                {
                    green = (mBayer->element(i, j - 1) + mBayer->element(i - 1, j)) / 2;
                    red = mBayer->element(i - 1, j - 1);
                }
                else if (!i)
                {
                    green = (mBayer->element(i, j + 1) + mBayer->element(i + 1, j) + mBayer->element(i, j - 1)) / 3;
                    red = (mBayer->element(i + 1, j + 1) + mBayer->element(i + 1, j - 1)) / 2;
                }
                else if (!j)
                {
                    green = (mBayer->element(i, j + 1) + mBayer->element(i + 1, j) + mBayer->element(i - 1, j)) / 3;
                    red = (mBayer->element(i + 1, j + 1) + mBayer->element(i - 1, j + 1)) / 2;
                }
                else if (i == mBayer->h - 1)
                {
                    green = (mBayer->element(i, j - 1) + mBayer->element(i - 1, j) + mBayer->element(i, j + 1)) / 3;
                    red = (mBayer->element(i - 1, j + 1) + mBayer->element(i - 1, j - 1)) / 2;
                }
                else if (j == mBayer->w - 1)
                {
                    green = (mBayer->element(i, j - 1) + mBayer->element(i + 1, j) + mBayer->element(i - 1, j)) / 3;
                    red = (mBayer->element(i + 1, j - 1) + mBayer->element(i - 1, j - 1)) / 2;
                }
                else
                {
                    green = (mBayer->element(i, j + 1) + mBayer->element(i, j - 1) + mBayer->element(i + 1, j) + mBayer->element(i - 1, j)) / 4;
                    red = (mBayer->element(i + 1, j + 1) + mBayer->element(i - 1, j + 1) + mBayer->element(i + 1, j - 1) + mBayer->element(i - 1, j - 1)) / 4;
                }

                blue = mBayer->element(i, j);
            }
            result->setElement(i, j, RGBColor48(
                mCurve[clip((int64_t)((red - mBlack)*mScaleMul[0]), mDepth)],
                mCurve[clip((int64_t)((green - mBlack)*mScaleMul[1]), mDepth)],
                mCurve[clip((int64_t)((blue - mBlack)*mScaleMul[2]), mDepth)]
                ));
        }

    return result;
}

double* Debayer::scaleCoeffs()
{
    // TODO: implement 'apply'
    double* scale_mul = new double[4];
    for (int i = 0; i < 4; i++)
        scale_mul[i] = 1;

    // alias for ease of use
    MetaData &metadata = *mMetadata;
    // check if meta- and white balance data available
    if (mMetadata == nullptr ||
        !metadata["cam_mul"] || !metadata["cam_mul"][0] || !metadata["cam_mul"][2] || !metadata["pre_mul"] || !metadata["pre_mul"][0])
        // TODO: maybe apply the gray world hypothesis instead of doing nothing
        return scale_mul; // if not, return vector of 1's

    unsigned c;
    double dmin;

    if (metadata["cam_mul"][0] != -1)
        for (int i = 0; i < 4; i++)
            metadata["pre_mul"][i] = metadata["cam_mul"][i]; // clarify: why are we doing this?

    if (!metadata["pre_mul"][1]) metadata["pre_mul"][1] = 1;
    if (!metadata["pre_mul"][3]) metadata["pre_mul"][3] = metadata["pre_mul"][1];

    metadata["white"] -= metadata["black"];

    for (dmin = DBL_MAX, c = 0; c < 4; c++)
    {
        if (dmin > metadata["pre_mul"][c])
            dmin = metadata["pre_mul"][c];
    }

    double factor = ((1 << mDepth) - 1) / metadata["white"];
    for (int c = 0; c < 4; c++)
        scale_mul[c] = (metadata["pre_mul"][c] /= dmin)*factor;
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

    return scale_mul;
}

uint16_t* Debayer::gammaCurve(int mode, int imax)
{
    // this code is taken from LibRaw
    // TODO: rewrite?
    uint16_t *curve = new uint16_t[0x10000];
    for (int i = 0; i < 0x10000; i++)
        curve[i] = i;

    // alias
    MetaData &metadata = *mMetadata;

    // if no gamm coefficients are present (or valid), return no transform
    if (mMetadata == nullptr || !metadata["gamm"] || !(metadata["gamm"][0] || metadata["gamm"][1] || metadata["gamm"][2] || metadata["gamm"][3] || metadata["gamm"][4]))
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
    if (!mode--)
    {
        memcpy(metadata["gamm"].second, g, sizeof g);
        return curve;
    }
    for (i = 0; i < 0x10000; i++)
    {
        curve[i] = (1 << mDepth) - 1;
        if ((r = (double)i / imax) < 1)
            curve[i] = (1 << mDepth) * (mode
                ? (r < g[3] ? r*g[1] : (g[0] ? pow(r, g[0])*(1 + g[4]) - g[4] : log(r)*g[2] + 1))
                : (r < g[2] ? r / g[1] : (g[0] ? pow((r + g[4]) / (1 + g[4]), 1 / g[0]) : exp((r - 1) / g[2]))));
    }
    return curve;
}

RGB48Buffer* Debayer::toRGB48(Quality quality)
{
    RGB48Buffer *result = nullptr;
    preprocess();

    switch (quality)
    {
    case Nearest:
        result = nearest();
        break;

    default:
    case Bilinear:
        result = linear();
        break;
    }

    return result;
}

void Debayer::preprocess(bool overwrite)
{
    if (!overwrite && (mScaleMul != nullptr || mCurve != nullptr))
        return;

    // alias for ease of use
    MetaData &metadata = *this->mMetadata;
    int m_bits = this->mMetadata != nullptr &&  metadata["bits"] ? metadata["bits"][0] : mDepth;
    int shift = m_bits - mDepth;

    mBlack = this->mMetadata != nullptr &&  metadata["black"] ? metadata["black"][0] : 0;
    int m_white = this->mMetadata != nullptr && metadata["white"] ? metadata["white"][0] : (1 << mDepth) - 1;
    int m_twhite = this->mMetadata != nullptr && metadata["t_white"] ? (int)metadata["t_white"][0] : m_white;
    int t_white = (shift < 0 ? m_twhite << -shift : m_twhite >> shift);

    // (re)calculate white balance coefficients
    if (overwrite && mScaleMul != nullptr)
        delete[] mScaleMul;
    if (overwrite || mScaleMul == nullptr)
        mScaleMul = scaleCoeffs();

    // (re)calculate gamma correction
    if (overwrite && mCurve != nullptr)
        delete[] mCurve;
    if (overwrite || mCurve == nullptr)
        mCurve = gammaCurve(2, t_white);
}

Debayer::~Debayer()
{
}
