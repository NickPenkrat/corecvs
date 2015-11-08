#include "debayer.h"
#include <limits>
#include "rgbConverter.h"

Debayer::Debayer(G12Buffer *bayer, int depth, MetaData *metadata)
    : mBayer(bayer)
    , mDepth(depth)
    , mMetadata(metadata)
{}

Debayer::~Debayer()
{
    deletearr_safe(mCurve);
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
    {
        for (int j = 0; j < mBayer->w; j += 2)
        {
            for (int k = 0; k < 2; k++)
            {
                for (int l = 0; l < 2; l++)
                {
                    int pxshift = (l ^ (l & 1));
                    // i don't know how i came to this
                    // green1 for even rows, green2 for odd
                    red   = mBayer->element(i +           swapRows, (j + pxshift)           ^  swapCols);
                    green = mBayer->element(i + (1 - k) ^ swapRows, (j + pxshift) + (1 - k) ^ !swapCols);
                    blue  = mBayer->element(i +          !swapRows, (j + pxshift)           ^ !swapCols);

                    result->element(i + k, j + l) = {
                        mCurve[clip((int64_t)((red - mBlack) * mScaleMul[0]), mDepth)],
                        mCurve[clip((int64_t)((green - mBlack) * mScaleMul[1]), mDepth)],
                        mCurve[clip((int64_t)((blue - mBlack) * mScaleMul[2]), mDepth)]
                    };
                }
        }
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
                    int color = colorFromBayerPos(k, l);

                    switch (color)
                    {
                    case 0: // red

                        // conventionally, y element is first and x is second when it comes to element getters in buffers, but in vectors it's actually inverse
                        // TODO: should I comply with the variable names and use vectors of the form { x, y } (used currently) or reverse variable order (change to { y, x })?
                        // the latter has a better readability while the former aims to avoid confusion and clutter in coordinates
                        // known color
                        red = mBayer->element(i + k, j + l);

                        // interpolated colors
                        green = weightedBayerAvg({ { j + l,     i + k - 1 }, { j + l,     i + k + 1 }, { j + l - 1, i + k     }, { j + l + 1, i + k     } });
                        blue  = weightedBayerAvg({ { j + l - 1, i + k - 1 }, { j + l + 1, i + k - 1 }, { j + l - 1, i + k + 1 }, { j + l + 1, i + k + 1 } });
                        break;
                    case 1: // green1
                        green = mBayer->element(i + k, j + l);

                        red  = weightedBayerAvg({ { j + l - 1, i + k     }, { j + l + 1, i + k     } });
                        blue = weightedBayerAvg({ { j + l,     i + k - 1 }, { j + l    , i + k + 1 } });
                        break;
                    case 2: // green2
                        green = mBayer->element(i + k, j + l);

                        red  = weightedBayerAvg({ { j + l,     i + k - 1 }, { j + l,     i + k + 1 } });
                        blue = weightedBayerAvg({ { j + l - 1, i + k     }, { j + l + 1, i + k     } });
                        break;
                    case 3: // blue
                        blue = mBayer->element(i + k, j + l);

                        red   = weightedBayerAvg({ { j + l - 1, i + k - 1 }, { j + l + 1, i + k - 1 }, { j + l - 1, i + k + 1 }, { j + l + 1, i + k + 1 } });
                        green = weightedBayerAvg({ { j + l,     i + k - 1 }, { j + l,     i + k + 1 }, { j + l - 1, i + k     }, { j + l + 1, i + k     } });
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

#define min(a,b) ( (a)<(b)?  (a) : (b) )
#define max(a,b) ( (a)>(b)?  (a) : (b) )
#define abs(x)   ( (x)>0.0?  (x) : (-(x)) )
#define sqr(x)   ( (x)*(x) )

    // allocate buffers for two directions
    G12Buffer *green_h = new G12Buffer(mBayer->h, mBayer->w, false);
    G12Buffer *green_v = new G12Buffer(mBayer->h, mBayer->w, false);
    RGB48Buffer *rgb_v = new RGB48Buffer(mBayer->h, mBayer->w, true);
    RGB48Buffer *rgb_h = new RGB48Buffer(mBayer->h, mBayer->w, true);
    float(*Lab_h)[3] = new float[mBayer->h*mBayer->w][3];
    float(*Lab_v)[3] = new float[mBayer->h*mBayer->w][3];

    const vector<int> filter = { -1, 2, 2, 2, -1 };

    uint16_t cur = 0;
    int32_t val = 0;

    // interpolate green
    for (int i = 0; i < mBayer->h; i += 2)
    {
        for (int j = 0; j < mBayer->w; j += 2)
        {
            for (int k = 0; k < 2 && i + k < mBayer->h; k++)
            {
                for (int l = 0; l < 2 && j + l < mBayer->w; l++)
                {
                    uint8_t color = colorFromBayerPos(k, l);

                    switch (color)
                    {
                        // green pixel, use bayer value
                    default:
                    case 1:
                    case 2:
                        cur = clip(mBayer->element(i + k, j + l), mDepth);
                        green_h->element(i + k, j + l) = cur;
                        green_v->element(i + k, j + l) = cur;
                        break;

                        // non-green pixel, interpolate
                    case 0:
                    case 3:
                        // apply low-pass filter with coefficients -1/4,1/2,1/2,1/2,-1/4 and clamp the resulting green value by its two neighbours
                        // the coefficients are close to these of Hirakawa & Parks' optimal filter
                        // horizontal
                        val = weightedBayerAvg({ { j + l - 2, i + k }, { j + l - 1, i + k }, { j + l, i + k }, { j + l + 1, i + k }, { j + l + 2, i + k } }, filter) / 4;
                        val = clamp(val, weightedBayerAvg({ j + l - 1, i + k }), weightedBayerAvg({ j + l + 1, i + k }));
                        green_h->element(i + k, j + l) = clip(val, mDepth);

                        //vertical
                        val = weightedBayerAvg({ { j + l, i + k - 2 }, { j + l, i + k - 1 }, { j + l, i + k }, { j + l, i + k + 1 }, { j + l, i + k + 2 } }, filter) / 4;
                        val = clamp(val, weightedBayerAvg({ j + l, i + k - 1 }), weightedBayerAvg({ j + l, i + k + 1 }));
                        green_v->element(i + k, j + l) = clip(val, mDepth);
                        break;
                    }
                }
            }
        }
    }

    // aliases to vertical and horizontal buffers
    G12Buffer *green = green_v;
    RGB48Buffer *rgb = rgb_v;
    float(*Lab)[3] = Lab_v;

    RGBColor48 pixel;

    // TODO: the following may hardly be considered readable by normal people
    // please kill me if i decide to write like this again
    // TODO: rewrite all _h/_v stuff asap

    // interpolate red and blue first vertically, then horizontally
    for (int i = 0; i < 2; i++, green = green_h, rgb = rgb_h, Lab = Lab_h)
    {
        for (int i = 0; i < mBayer->h - 1; i += 2)
        {
            for (int j = 0; j < mBayer->w - 1; j += 2)
            {
                for (int k = 0; k < 2; k++)
                {
                    for (int l = 0; l < 2; l++)
                    {
                        uint8_t color = colorFromBayerPos(k, l, false);

                        pixel[1] = green->element(i + k, j + l);

                        if (color == 1)
                        {
                            uint8_t interp_c = colorFromBayerPos(k + 1, l, false);

                            val = pixel[1] + ((weightedBayerAvg({ j + l - 1, i + k }) + weightedBayerAvg({ j + l + 1, i + k })
                                - green->element(i + k, j + l - 1) - green->element(i + k, j + l + 1)) >> 1);
                            pixel[interp_c] = clip(val, mDepth);

                            val = pixel[1] + ((weightedBayerAvg({ j + l, i + k - 1 }) + weightedBayerAvg({ j + l, i + k + 1 })
                                - green->element(i + k - 1, j + l) - green->element(i + k + 1, j + l)) >> 1);
                            pixel[2 - interp_c] = clip(val, mDepth);
                        }
                        else
                        {
                            val = green->element(i + k, j + l) + ((weightedBayerAvg({ j + l - 1, i + k - 1 }) + weightedBayerAvg({ j + l + 1, i + k - 1 })
                                + weightedBayerAvg({ j + l - 1, i + k + 1 }) + weightedBayerAvg({ j + l + 1, i + k + 1 })
                                - green->element(i + k - 1, j + l - 1) - green->element(i + k - 1, j + l + 1)
                                - green->element(i + k + 1, j + l - 1) - green->element(i + k + 1, j + l + 1) + 1) >> 2);
                            pixel[color] = clip(val, mDepth);
                            pixel[2 - color] = clip(mBayer->element(i + k, j + l), mDepth);
                        }
                        rgb->element(i + k, j + l) = pixel;
                        RGBConverter::rgb2Lab(pixel, Lab[(i + k)*mBayer->w + j + l]);
                    }
                }
            }
        }
    }

    // free some memory
    delete_safe(green_v);
    delete_safe(green_h);

    float *homo_v = new float[mBayer->h*mBayer->w];
    float *homo_h = new float[mBayer->h*mBayer->w];

    float *homo = homo_v;
    Lab = Lab_v;

    RGB48Buffer *result = new RGB48Buffer(mBayer->h, mBayer->w, false);


    // build homogeneity maps using cielab metric
    for (int i = 1; i < mBayer->h - 1; i++)
    {
        for (int j = 1; j < mBayer->w - 1; j++)
        {
            homo = homo_v;
            Lab = Lab_v;
            int offset = i*mBayer->w + j;

            // luminance difference in 4 directions for 2 images
            float dl[2][4];

            // chrominance difference in 4 directions for 2 images
            float dc[2][4];

            for (int d = 0; d < 2; d++, Lab = Lab_h)
            {
                int idx = 0;
                for (int k = -1; k < 2; k += 2, idx++)
                {
                    int shift_a = k*mBayer->w;
                    dl[d][idx] = abs(Lab[offset][0] - Lab[offset + shift_a][0]);
                    dc[d][idx] = sqr(Lab[offset][1] - Lab[offset + shift_a][1]) + sqr(Lab[offset][2] - Lab[offset + shift_a][2]);
                }
                for (int l = -1; l < 2; l += 2, idx++)
                {
                    int shift_b = l;
                    dl[d][idx] = abs(Lab[offset][0] - Lab[offset + shift_b][0]);
                    dc[d][idx] = sqr(Lab[offset][1] - Lab[offset + shift_b][1]) + sqr(Lab[offset][2] - Lab[offset + shift_b][2]);
                }
            }

            float leps = min(max(dl[0][0], dl[0][1]),
                max(dl[1][2], dl[1][3]));
            float abeps = min(max(dc[0][0], dc[0][1]),
                max(dc[1][2], dc[1][3]));

            for (int d = 0; d < 2; d++, homo = homo_h)
            {
                int homo_val = 0;
                for (int j = 0; j < 4; j++)
                {
                    if (dl[d][j] <= leps && dc[d][j] <= abeps)
                    {
                        homo_val++;
                    }
                }
                homo[offset] = homo_val;
            }
        }
    }

    // select homogeneous pixels
    int homo_cur[2];
    for (int i = 0; i < mBayer->h; i++)
    {
        for (int j = 0; j < mBayer->w; j++)
        {
            homo_cur[0] = homo_cur[1] = 0;
            if (i <= 1 || j <= 1 || i >= mBayer->h - 2 || j >= mBayer->w - 2)
            {
                result->element(i, j) = (rgb_v->element(i, j) + rgb_h->element(i, j)) / 2;
            }
            else
            {
                for (int k = i - 1; k <= i + 1; k++)
                    for (int l = j - 1; l <= j + 1; l++)
                    {
                        homo = homo_v;
                        for (int d = 0, offset = k*mBayer->w + l; d < 2; d++, homo = homo_h)
                            homo_cur[d] += homo[offset];
                    }

                if (homo_cur[0] > homo_cur[1])
                    result->element(i, j) = rgb_v->element(i, j);
                else if (homo_cur[0] < homo_cur[1])
                    result->element(i, j) = rgb_h->element(i, j);
                else
                    result->element(i, j) = (rgb_v->element(i, j) + rgb_h->element(i, j)) / 2;
            }
        }
    }


    deletearr_safe(Lab_v);
    deletearr_safe(Lab_h);
    deletearr_safe(homo_v);
    deletearr_safe(homo_h);
    delete_safe(rgb_v);
    delete_safe(rgb_h);

    return result;
#undef min
#undef max
#undef abs
#undef sqr
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

void Debayer::gammaCurve(uint16_t *curve, int imax)
{
    // this code is taken from LibRaw
    // TODO: rewrite?

    // initialize curve as a straight line (no correction)
    for (int i = 0; i < 0x10000; i++)
        curve[i] = i;

    if (mMetadata == nullptr)
        return;
    // alias
    MetaData &metadata = *mMetadata;

    // if no gamm coefficients are present (or valid), return no transform
    if (metadata["gamm"].empty() || !(metadata["gamm"][0] || metadata["gamm"][1] || metadata["gamm"][2] || metadata["gamm"][3] || metadata["gamm"][4]))
        return;

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
}

RGB48Buffer* Debayer::toRGB48(Method method)
{
    preprocess();

    RGB48Buffer *result = nullptr;
    switch (method)
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
    if (overwrite || mCurve == nullptr) {
        deletearr_safe(mCurve);
        mCurve = new uint16_t[0x10000];
        gammaCurve(mCurve, t_white);
    }
}

int32_t Debayer::weightedBayerAvg(Vector2d32 coords)
{
    return weightedBayerAvg(vector<Vector2d32>{coords});
}

int32_t Debayer::weightedBayerAvg(vector<Vector2d32> coords, vector<int> coeffs)
{
    int32_t result = 0;
    uint16_t div = 0;

    bool useCoeffs = coeffs.size() >= coords.size();

    for (int i = 0; i < coords.size(); i++)
    {
        if (coords[i].x() >= 0 && coords[i].y() >= 0 && coords[i].x() < mBayer->w && coords[i].y() < mBayer->h)
        {
            div++;
            if (useCoeffs)
                result += mBayer->element(coords[i])*coeffs[i];
            else
                result += mBayer->element(coords[i]);
        }
    }
    if (div == 0)
        return 0;
    else
        return result / div;
}

int32_t Debayer::clamp(int32_t x, int32_t a, int32_t b)
{
    if (a > b)
        SwapInts<int32_t>(a, b);
    if (x < a)
        return a;
    if (x > b)
        return b;
    return x;
}

inline uint8_t Debayer::colorFromBayerPos(int i, int j, bool rggb)
{
    if (rggb)   // r, g1, g2, b
        return   (j ^ (mBayerPos & 1)) & 1 | (((i ^ ((mBayerPos & 2) >> 1)) & 1) << 1);
    else        // r, g, b
        return (((j ^ (mBayerPos & 1)) & 1 | (((i ^ ((mBayerPos & 2) >> 1)) & 1) << 1)) + 1) >> 1;
}

inline uint16_t Debayer::clip(int32_t x, int depth)
{
    const uint16_t maximum = (1 << depth) - 1;
    if (x < 0)
        return 0;
    if (x > maximum || x >= (1 << 16))
        return maximum;
    return (uint16_t)x;
}
