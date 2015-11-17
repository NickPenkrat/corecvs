#include "debayer.h"
#include <limits>
#include "rgbConverter.h"
#include <fftw/fftw3.h>
#include <mkl_dfti.h>
#include "ppmLoader.h"
#include <complex>

Debayer::Debayer(G12Buffer *bayer, int depth, int bayerPos, MetaData *metadata)
    : mBayer(bayer)
    , mDepth(depth)
    , mBayerPos(bayerPos)
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
    uint16_t red = 0, green = 0, blue = 0;

    // RGGB
    // TODO: write this to metadata
    //int bpos = 0;
    //int swapCols = bpos & 1;
    //int swapRows = (bpos & 2) >> 1;
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

int compare(const void * a, const void * b)
{
    return (*(int*)a - *(int*)b);
}

RGB48Buffer* Debayer::ahd()
{

#define MIN(a,b) ( (a)<(b)?  (a) : (b) )
#define MAX(a,b) ( (a)>(b)?  (a) : (b) )
#define ABS(x)   ( (x)>0.0?  (x) : (-(x)) )
#define SQR(x)   ( (x)*(x) )

    // allocate buffers for two directions
    G12Buffer *green[2] = {
        new G12Buffer(mBayer->h, mBayer->w, false),
        new G12Buffer(mBayer->h, mBayer->w, false)
    };
    RGB48Buffer *rgb[2] = {
        new RGB48Buffer(mBayer->h, mBayer->w, true),
        new RGB48Buffer(mBayer->h, mBayer->w, true)
    };

    float(*Lab[2])[3] = {
        new float[mBayer->h*mBayer->w][3],
        new float[mBayer->h*mBayer->w][3]
    };

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
                        green[0]->element(i + k, j + l) = cur;
                        green[1]->element(i + k, j + l) = cur;
                        break;

                        // non-green pixel, interpolate
                    case 0:
                    case 3:

                        // apply low-pass filter with coefficients -1/4,1/2,1/2,1/2,-1/4 and clamp the resulting green value by its two neighbours
                        // the coefficients are close to these of Hirakawa & Parks' optimal filter
                        //vertical
                        val = weightedBayerAvg({ { j + l, i + k - 2 }, { j + l, i + k - 1 }, { j + l, i + k }, { j + l, i + k + 1 }, { j + l, i + k + 2 } }, filter) / 4;
                        val = clamp(val, weightedBayerAvg({ j + l, i + k - 1 }), weightedBayerAvg({ j + l, i + k + 1 }));
                        green[0]->element(i + k, j + l) = clip(val, mDepth);

                        // horizontal
                        val = weightedBayerAvg({ { j + l - 2, i + k },{ j + l - 1, i + k },{ j + l, i + k },{ j + l + 1, i + k },{ j + l + 2, i + k } }, filter) / 4;
                        val = clamp(val, weightedBayerAvg({ j + l - 1, i + k }), weightedBayerAvg({ j + l + 1, i + k }));
                        green[1]->element(i + k, j + l) = clip(val, mDepth);
                        break;
                    }
                }
            }
        }
    }

    RGBColor48 pixel;

    // interpolate red and blue first vertically, then horizontally
    for (int d = 0; d < 2; d++)
    {
        for (int i = 1; i < mBayer->h - 1; i += 2)
        {
            for (int j = 1; j < mBayer->w - 1; j += 2)
            {
                for (int k = 0; k < 2; k++)
                {
                    for (int l = 0; l < 2; l++)
                    {
                        pixel[1] = green[d]->element(i + k, j + l);

                        uint8_t color = colorFromBayerPos(i + k, j + l, false);

                        if (color == 1)
                        {
                            uint8_t row_c = colorFromBayerPos(i + k, j + l + 1, false);

                            // C = G + LP(C' - G'), where C is sought colour
                            // LP is in fact the average
                            val = pixel[1] + ((weightedBayerAvg({ j + l - 1, i + k }) - green[d]->element(i + k, j + l - 1)
                                             + weightedBayerAvg({ j + l + 1, i + k }) - green[d]->element(i + k, j + l + 1)) / 2);
                            
                            // logically, this should be pixel[row_c], but our pixels are BGR, so this is inverted
                            pixel[2 - row_c] = clip(val, mDepth);

                            val = pixel[1] + ((weightedBayerAvg({ j + l, i + k - 1 }) - green[d]->element(i + k - 1, j + l)
                                             + weightedBayerAvg({ j + l, i + k + 1 }) - green[d]->element(i + k + 1, j + l)) / 2);
                            pixel[row_c] = clip(val, mDepth);
                        }
                        else
                        {
                            // known colour: inverted (same as above)
                            pixel[2 - color] = clip(mBayer->element(i + k, j + l), mDepth);

                            // interpolate colour using greens diagonally
                            // this is not intuitive, but directly follows from the aforementioned equation
                            val = pixel[1] +
                                    ((weightedBayerAvg({ j + l - 1, i + k - 1 }) - green[d]->element(i + k - 1, j + l - 1)
                                    + weightedBayerAvg({ j + l + 1, i + k - 1 }) - green[d]->element(i + k + 1, j + l + 1)
                                    + weightedBayerAvg({ j + l - 1, i + k + 1 }) - green[d]->element(i + k - 1, j + l + 1)
                                    + weightedBayerAvg({ j + l + 1, i + k + 1 }) - green[d]->element(i + k + 1, j + l - 1)
                                     
                                     ) / 4);
                            pixel[color] = clip(val, mDepth);
                        }
                        rgb[d]->element(i + k, j + l) = pixel;
                        RGBConverter::rgb2Lab(pixel, Lab[d][(i + k)*mBayer->w + j + l]);
                    }
                }
            }
        }
    }

    // free some memory
    delete_safe(green[0]);
    delete_safe(green[1]);

    float *homo[2] = {
        new float[mBayer->h*mBayer->w],
        new float[mBayer->h*mBayer->w]
    };

    RGB48Buffer *result = new RGB48Buffer(mBayer->h, mBayer->w, false);


    // build homogeneity maps using cielab metric
    for (int i = 0; i < mBayer->h; i++)
    {
        for (int j = 0; j < mBayer->w; j++)
        {
            int offset = i*mBayer->w + j;

            // luminance difference in 4 directions for 2 images
            float dl[2][4];

            // chrominance difference in 4 directions for 2 images
            float dc[2][4];

            for (int d = 0; d < 2; d++)
            {
                int idx = 0;
                for (int k = -1; k < 2; k += 2, idx++)
                {
                    int shift_a = k*mBayer->w;
                    if (offset + shift_a < 0 || offset + shift_a >= mBayer->h*mBayer->w)
                        continue;
                    dl[d][idx] = ABS(Lab[d][offset][0] - Lab[d][offset + shift_a][0]);
                    dc[d][idx] = SQR(Lab[d][offset][1] - Lab[d][offset + shift_a][1]) + SQR(Lab[d][offset][2] - Lab[d][offset + shift_a][2]);
                }
                for (int l = -1; l < 2; l += 2, idx++)
                {
                    int shift_b = l;
                    if (offset + shift_b < 0 || offset + shift_b >= mBayer->h*mBayer->w)
                        continue;
                    dl[d][idx] = ABS(Lab[d][offset][0] - Lab[d][offset + shift_b][0]);
                    dc[d][idx] = SQR(Lab[d][offset][1] - Lab[d][offset + shift_b][1]) + SQR(Lab[d][offset][2] - Lab[d][offset + shift_b][2]);
                }
            }

            // min { max { DIFF(x, neighbor) } } for all neighbors from B(x, 1)
            // the luminance and chrominance deviations are defined as maximal deviation in any direction
            // we must choose minimal deviations to count homogenous pixels

            // VERSION A - proposed by Hirakawa & Parks, produces noticeable artifacts on cm_lighthouse.pgm
            //float eps_l = MIN(MAX(dl[0][0], dl[0][1]), MAX(dl[1][2], dl[1][3]));
            float eps_c = MIN(MAX(dc[0][0], dc[0][1]), MAX(dc[1][2], dc[1][3]));

            // VERSION B - extended, produces less artifacts on cm_lighthouse.pgm, but instead produces weak zipper on test_debayer.pgm
            float eps_l = MIN(MAX(MAX(dl[0][0], dl[0][1]), MAX(dl[0][2], dl[0][3])), MAX(MAX(dl[1][0], dl[1][1]), MAX(dl[1][2], dl[1][3])));
            //float eps_c = MIN(MAX(MAX(dc[0][0], dc[0][1]), MAX(dc[0][2], dc[0][3])), MAX(MAX(dc[1][0], dc[1][1]), MAX(dc[1][2], dc[1][3])));

            for (int d = 0; d < 2; d++)
            {
                int homo_val = 0;
                for (int j = 0; j < 4; j++)
                {
                    if (dl[d][j] <= eps_l && dc[d][j] <= eps_c)
                    {
                        homo_val++;
                    }
                }
                homo[d][offset] = homo_val;
            }
        }
    }

    int32_t *rgbDiff[4] = {
        new int32_t[mBayer->h*mBayer->w], // R - G
        new int32_t[mBayer->h*mBayer->w], // B - G
    };

    // select homogeneous pixels
    int homo_cur[2];
    for (int i = 0; i < mBayer->h; i++)
    {
        for (int j = 0; j < mBayer->w; j++)
        {
            homo_cur[0] = homo_cur[1] = 0;
            if (i <= 1 || j <= 1 || i >= mBayer->h - 2 || j >= mBayer->w - 2)
            {
                result->element(i, j) = (rgb[0]->element(i, j) + rgb[1]->element(i, j)) / 2;
            }
            else
            {
                for (int k = i - 1; k <= i + 1; k++)
                    for (int l = j - 1; l <= j + 1; l++)
                    {
                        for (int d = 0, offset = k*mBayer->w + l; d < 2; d++)
                            homo_cur[d] += homo[d][offset];
                    }

                if (homo_cur[0] > homo_cur[1])
                    result->element(i, j) = rgb[0]->element(i, j);
                else if (homo_cur[0] < homo_cur[1])
                    result->element(i, j) = rgb[1]->element(i, j);
                else
                    result->element(i, j) = (rgb[0]->element(i, j) + rgb[1]->element(i, j)) / 2;
            }
            rgbDiff[0][i*mBayer->w + j] = result->element(i, j).r() - result->element(i, j).g();
            rgbDiff[1][i*mBayer->w + j] = result->element(i, j).b() - result->element(i, j).g();
        }
    }


    deletearr_safe(Lab[0]);
    deletearr_safe(Lab[1]);
    deletearr_safe(homo[0]);
    deletearr_safe(homo[1]);
    delete_safe(rgb[0]);
    delete_safe(rgb[1]);

    // apply median filter to rgb result
    // filter radius
    // radius of 1 gives nice results
    // radius of 2 gives less false color artifacts for some images, but the colors become somewhat degraded and blurred for other images
    const int radius = 1;
    // filter size - do not change
    const int size = SQR(2 * radius + 1);

    // median filter pass count, no difference except for running time observed between 1 and 2, more than 2 is redundant
    const int passes = 2;

    for (int p = 0; p < passes; p++)
        for (int i = radius + 1; i < mBayer->h - radius - 1; i++)
        {
            for (int j = radius + 1; j < mBayer->w - radius - 1; j++)
            {
                int32_t window[2][size];
                for (int c = 0; c < 2; c++)
                {
                    int idx = 0;
                    for (int k = i - radius; k <= i + radius; k++)
                        for (int l = j - radius; l <= j + radius; l++)
                            window[c][idx++] = rgbDiff[c][k*mBayer->w + l];
                    qsort(window[c], size, sizeof(window[c][0]), compare);

                }

                uint32_t r = window[0][4] + result->element(i, j).g();
                uint32_t b = window[1][4] + result->element(i, j).g();
                uint32_t g = (r + b - window[0][4] - window[1][4]) / 2;
                result->element(i, j) = RGBColor48(
                    clip(r*mScaleMul[0], mDepth), 
                    clip(g*mScaleMul[1], mDepth),
                    clip(b*mScaleMul[2], mDepth)
                );

                rgbDiff[0][i*mBayer->w + j] = r - g;
                rgbDiff[1][i*mBayer->w + j] = b - g;
            }
        }

    deletearr_safe(rgbDiff[0]);
    deletearr_safe(rgbDiff[1]);
    deletearr_safe(rgbDiff[2]);
    deletearr_safe(rgbDiff[3]);

    return result;

#undef min
#undef max
#undef abs
#undef sqr
}

RGB48Buffer* Debayer::fourier()
{
    DFTI_DESCRIPTOR_HANDLE descriptor;
    MKL_LONG status;

    

    uint h = mBayer->h;
    uint w = mBayer->w;

    fftw_complex* in_r, *in_g, *in_b, *out_r, *out_g, *out_b;

    in_r = fftw_alloc_complex(h*w);
    in_g = fftw_alloc_complex(h*w);
    in_b = fftw_alloc_complex(h*w);

    out_r = fftw_alloc_complex(h*w);
    out_g = fftw_alloc_complex(h*w);
    out_b = fftw_alloc_complex(h*w);

    RGB48Buffer *out_fourier = new RGB48Buffer(h, w, false),
                *out_orig    = new RGB48Buffer(h, w, false);

    for (int i = 0; i < h; i++)
        for (int j = 0; j < w; j++)
        {
            int offset = i*w + j;
            int color = colorFromBayerPos(i, j, false);
            switch (color)
            {
            case 0:
                in_r[offset][0] = (i < mBayer->h && j < mBayer->w) ? mBayer->data[i*mBayer->w + j] : 0, 0;
                break;
            case 1:
                in_g[offset][0] = (i < mBayer->h && j < mBayer->w) ? mBayer->data[i*mBayer->w + j] : 0, 0;
                break;
            case 2:
                in_b[offset][0] = (i < mBayer->h && j < mBayer->w) ? mBayer->data[i*mBayer->w + j] : 0, 0;
                break;
            }
            out_orig->element(i, j) = RGBColor48(in_r[i*mBayer->w + j][0], in_g[i*mBayer->w + j][0], in_b[i*mBayer->w + j][0]);
        }

    fftw_plan plan = fftw_plan_dft_2d(h, w, in_r, out_r, FFTW_FORWARD, 0);
    fftw_execute(plan);
    fftw_execute_dft(plan, in_g, out_g);
    fftw_execute_dft(plan, in_b, out_b);
    fftw_destroy_plan(plan);

    int newsize = sqrt(h*w);
    G12Buffer *val_r = new G12Buffer(h, w, false),
              *val_g = new G12Buffer(h, w, false),
              *val_b = new G12Buffer(h, w, false);
    for (int i = 0; i < h; i++)
        for (int j = 0; j < w; j++)
        {
            val_r->element(i, j) = clip(sqrtf(SQR(out_r[i*w + j][0]) + SQR(out_r[i*w + j][1]))/10);
            val_g->element(i, j) = clip(sqrtf(SQR(out_g[i*w + j][0]) + SQR(out_g[i*w + j][1]))/10);
            val_b->element(i, j) = clip(sqrtf(SQR(out_b[i*w + j][0]) + SQR(out_b[i*w + j][1]))/10);
        }

    PPMLoader().save("four_out.pgm", val_g);
    PPMLoader().save("four_in.ppm", out_orig);
    return nullptr;
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
    if ((metadata["cam_mul"].empty() || metadata["cam_mul"][0] == 0 || metadata["cam_mul"][2] == 0) &&
        (metadata["pre_mul"].empty() || metadata["pre_mul"][0] == 0 || metadata["pre_mul"][2] == 0))
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

    if (metadata["pre_mul"].empty())
        metadata["pre_mul"] = MetaValue(4, 1.0);

    if (!metadata["cam_mul"].empty() && metadata["cam_mul"][0] != -1)
    {
        for (int i = 0; i < 3; i++)
            metadata["pre_mul"][i] = metadata["cam_mul"][i];
    }
    // if green scale coefficient is not available, set it to 1
    if (metadata["pre_mul"][1] == 0)
        metadata["pre_mul"][1] = 1;

    // black must be subtracted from the image, so we adjust maximum white level here
    if(!metadata["white"].empty() && !metadata["black"].empty())
        metadata["white"][0] -= metadata["black"][0];

    // normalize pre_mul
    for (dmin = std::numeric_limits<double>::max(), c = 0; c < 3; c++)
    {
        if (dmin > metadata["pre_mul"][c])
            dmin = metadata["pre_mul"][c];
    }

    // scale colors to the desired bit-depth
    double factor;

    if (!metadata["white"].empty())
        factor = ((1 << mDepth) - 1) / metadata["white"][0];
    else if (!metadata["bits"].empty())
        factor = ((1 << mDepth) - 1) / metadata["bits"][0];
    else
        factor = 1.0;

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
    // TODO: rewrite it?

    // initialize curve as a straight line (no correction)
    for (int i = 0; i < 0x10000; i++)
        curve[i] = i;

    if (mMetadata == nullptr)
        return;
    // alias
    MetaData &metadata = *mMetadata;

    // if no gamma coefficients are present (or valid), return no transform
    auto& gammData = metadata["gamm"];
    if (gammData.empty() ||
        (gammData.size() == 5) && !(gammData[0] || gammData[1] || gammData[2] || gammData[3] || gammData[4]))
        return;

    int i;
    double r, g[6], bnd[2] = { 0, 0 };

    g[0] = gammData[0];
    g[1] = gammData[1];
    g[2] = g[3] = g[4] = 0;
    bnd[g[1] >= 1] = 1;

    if (g[1] && (g[1] - 1) * (g[0] - 1) <= 0)
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
    case AHD:
        result = ahd();
        break;
    }

    return result;
}

void Debayer::preprocess(bool overwrite)
{
    // recalculate white balance coefficients
    scaleCoeffs();

    int t_white = 0;

    if (mMetadata != nullptr && !mMetadata->empty() && (overwrite || mCurve == nullptr))
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
        if (coords[i].x() < 0)
            coords[i].x() = -coords[i].x();

        if (coords[i].y() < 0)
            coords[i].y() = -coords[i].y();

        if (coords[i].x() >= mBayer->w)
        {
            coords[i].x() = 2 * mBayer->w - coords[i].x() - 1;
        }

        if (coords[i].y() >= mBayer->h)
        {
            coords[i].y() = 2 * mBayer->h - coords[i].y() - 1;
        }

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
