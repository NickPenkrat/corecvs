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
    // RGGB
    // TODO: write this to metadata
    int bpos = 0;
    uint16_t red = 0, green = 0, blue = 0;

    int swapCols = bpos & 1;
    int swapRows = bpos & 2;
    RGB48Buffer *result = new RGB48Buffer(mBayer->h, mBayer->w, false);

    for (int i = 0; i < mBayer->h; i += 2)
        for (int j = 0; j < mBayer->w; j += 2)
        {
            for (int k = 0; k < 2; k++)
                for (int l = 0; l < 2; l++)
                {
                    // i don't know how i came to this
                    // swapRows, swapCols -> 0 "red"
                    red = mBayer->element(i + swapRows, j + swapCols);
                    // green1 for even rows, green2 for odd
                    green = mBayer->element(i + !k^swapRows, j + !k^!swapCols);
                    // !swapRows, !swapCols -> 2 "blue"
                    blue = mBayer->element(i + !swapRows, j + !swapCols);

                    result->setElement(i + k, j + l, RGBColor48(
                        mCurve[clip((int64_t)((red - mBlack)*mScaleMul[0]), mDepth)],
                        mCurve[clip((int64_t)((green - mBlack)*mScaleMul[1]), mDepth)],
                        mCurve[clip((int64_t)((blue - mBlack)*mScaleMul[2]), mDepth)]
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
    int swapRows = bpos & 2;
    RGB48Buffer *result = new RGB48Buffer(mBayer->h, mBayer->w, false);

    for (int i = 0; i < mBayer->h; i += 2)
        for (int j = 0; j < mBayer->w; j += 2)
        {
            for (int k = 0; k < 2; k++)
                for (int l = 0; l < 2; l++)
                {
                    int color = (l + swapCols) % 2 + ((k + swapRows) % 2) * 2;

                    switch (color)
                    {
                    case 0: // red
                        red = mBayer->element(i + k, j + l);
                        green = clampCoord(mBayer, Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1), Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k));
                        blue = clampCoord(mBayer, Vector2d32(j + l - 1, i + k - 1), Vector2d32(j + l + 1, i + k - 1), Vector2d32(j + l - 1, i + k + 1), Vector2d32(j + l + 1, i + k + 1));
                        break;
                    case 1: // green1
                        red = clampCoord(mBayer, Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k));
                        green = mBayer->element(i + k, j + l);
                        blue = clampCoord(mBayer, Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1));
                        break;
                    case 2: // green2
                        red = clampCoord(mBayer, Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1));
                        green = mBayer->element(i + k, j + l);
                        blue = clampCoord(mBayer, Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k));
                        break;
                    case 3: // blue
                        red = clampCoord(mBayer, Vector2d32(j + l - 1, i + k - 1), Vector2d32(j + l + 1, i + k - 1), Vector2d32(j + l - 1, i + k + 1), Vector2d32(j + l + 1, i + k + 1));
                        green = clampCoord(mBayer, Vector2d32(j + l, i + k - 1), Vector2d32(j + l, i + k + 1), Vector2d32(j + l - 1, i + k), Vector2d32(j + l + 1, i + k));
                        blue = mBayer->element(i + k, j + l);
                        break;
                    }

                    result->setElement(i + k, j + l, RGBColor48(
                        mCurve[clip((int64_t)((red - mBlack)*mScaleMul[0]), mDepth)],
                        mCurve[clip((int64_t)((green - mBlack)*mScaleMul[1]), mDepth)],
                        mCurve[clip((int64_t)((blue - mBlack)*mScaleMul[2]), mDepth)]
                        ));
                }
        }

    return result;
}

double* Debayer::scaleCoeffs()
{
    double* scale_mul = new double[3];
    for (int i = 0; i < 3; i++)
        scale_mul[i] = 1;

    // check if metadata available
    if (mMetadata == nullptr)
        return scale_mul; // if not, return vector of 1's (no scaling)

    // alias for ease of use
    MetaData &metadata = *mMetadata;

    // check if metadata valid
    if (!metadata["cam_mul"] || !metadata["cam_mul"][0] || !metadata["cam_mul"][2] || !metadata["pre_mul"] || !metadata["pre_mul"][0])
    {
        // if white balance not available, do only scaling
        // white should be calculated before calling scaleCoeffs()
        if (metadata["white"])
        {
            double factor = ((1 << mDepth) - 1) / metadata["white"];

            for (int i = 0; i < 3; i++)
                scale_mul[i] = factor;

            return scale_mul;
        }
        else
            // TODO: maybe apply the gray world hypothesis instead of doing nothing
            return scale_mul;
    }

    unsigned c;
    double dmin;

    // if cam_mul coefficients are available directly, use them in future
    // if not, use pre_mul when available
    if (metadata["cam_mul"][0] != -1)
        for (int i = 0; i < 4; i++)
            metadata["pre_mul"][i] = metadata["cam_mul"][i];

    // if green scale coefficient is not available, set it to 1
    if (metadata["pre_mul"][1] == 0) metadata["pre_mul"][1] = 1;

    // black must be subtracted from the image, so we adjust maximum white level here
    metadata["white"] -= metadata["black"];

    // normalize pre_mul
    for (dmin = DBL_MAX, c = 0; c < 4; c++)
    {
        if (dmin > metadata["pre_mul"][c])
            dmin = metadata["pre_mul"][c];
    }

    // scale colors to the desired bit-depth
    double factor = ((1 << mDepth) - 1) / metadata["white"];
    for (int c = 0; c < 3; c++)
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

    // initialize curve as a straight line (no correction)
    uint16_t *curve = new uint16_t[0x10000];
    for (int i = 0; i < 0x10000; i++)
        curve[i] = i;

    if (mMetadata == nullptr)
        return curve;

    // alias
    MetaData &metadata = *mMetadata;

    // if no gamm coefficients are present (or valid), return no transform
    if (!metadata["gamm"] || !(metadata["gamm"][0] || metadata["gamm"][1] || metadata["gamm"][2] || metadata["gamm"][3] || metadata["gamm"][4]))
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

    // (re)calculate white balance coefficients
    if (overwrite && mScaleMul != nullptr)
        delete[] mScaleMul;
    if (overwrite || mScaleMul == nullptr)
        mScaleMul = scaleCoeffs();

    int t_white = 0;

    if (mMetadata != nullptr && (overwrite || mScaleMul == nullptr && mCurve == nullptr))
    {
        // alias for ease of use
        MetaData &metadata = *this->mMetadata;
        int m_bits = this->mMetadata != nullptr &&  metadata["bits"] ? metadata["bits"][0] : mDepth;
        int shift = m_bits - mDepth;

        mBlack = this->mMetadata != nullptr &&  metadata["black"] ? metadata["black"][0] : 0;
        int m_white = this->mMetadata != nullptr && metadata["white"] ? metadata["white"][0] : (1 << mDepth) - 1;
        int m_twhite = this->mMetadata != nullptr && metadata["t_white"] ? (int)metadata["t_white"][0] : m_white;
        t_white = (shift < 0 ? m_twhite << -shift : m_twhite >> shift);
    }
    // (re)calculate gamma correction
    if (overwrite)
        delete[] mCurve;
    if (overwrite || mCurve == nullptr)
        mCurve = gammaCurve(2, t_white);

}

uint16_t Debayer::clampCoord(G12Buffer* buf, Vector2d32 coord1, Vector2d32 coord2, Vector2d32 coord3 = Vector2d32(-1, -1), Vector2d32 coord4 = Vector2d32(-1, -1))
{
    uint16_t result = 0;
    uint16_t div = 0;

    if (coord1.x() >= 0 && coord1.y() >= 0 && coord1.x() < buf->w && coord1.y() < buf->h)
    {
        div++;
        result += buf->element(coord1);
    }

    if (coord2.x() >= 0 && coord2.y() >= 0 && coord2.x() < buf->w && coord2.y() < buf->h)
    {
        div++;
        result += buf->element(coord2);
    }

    if (coord3.x() >= 0 && coord3.y() >= 0 && coord3.x() < buf->w && coord3.y() < buf->h)
    {
        div++;
        result += buf->element(coord3);
    }

    if (coord4.x() >= 0 && coord4.y() >= 0 && coord4.x() < buf->w && coord4.y() < buf->h)
    {
        div++;
        result += buf->element(coord4);
    }

    if (div == 0)
        return 0;
    else
        return result / div;
}

Debayer::~Debayer()
{
    delete[] mCurve;
    delete[] mScaleMul;

    if (mMetadata != nullptr)
        for (MetaData::iterator i = mMetadata->begin(); i != mMetadata->end(); i++)
            delete[] i->second.second;

    delete mMetadata;
}

