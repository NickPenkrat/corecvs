#include "debayer.h"
#include <limits>
#include "rgbConverter.h"
#include <fftw/fftw3.h>
#include <mkl_dfti.h>
#include "ppmLoader.h"
#include <complex>

using std::pow;
using std::max;
using std::min;
using std::abs;

Debayer::Debayer(G12Buffer *bayer, int depth, int bayerPos, MetaData *metadata)
    : mBayer(bayer)
    , mDepth(depth)
    , mBayerPos(bayerPos)
    , mMetadata(metadata)
{
    mMaximum = (1 << depth) - 1;
}

Debayer::~Debayer()
{
    deletearr_safe(mCurve);
}

void Debayer::nearest(RGB48Buffer *result)
{
    // RGGB
    // TODO: write this to metadata
    int bpos = 0;
    uint16_t red = 0, green = 0, blue = 0;

    // swapCols inverts least significant bit for cols when set so RG/GB becomes GR/BG, etc.
    // swapRows does the same for rows
    int swapCols = bpos & 1;
    int swapRows = (bpos & 2) >> 1;

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
                        mCurve[clip((int64_t)((red   - mBlack) * mScaleMul[0]))],
                        mCurve[clip((int64_t)((green - mBlack) * mScaleMul[1]))],
                        mCurve[clip((int64_t)((blue  - mBlack) * mScaleMul[2]))]
                    };
                }
            }
        }
    }
}

void Debayer::linear(RGB48Buffer *result)
{
    uint16_t red = 0, green = 0, blue = 0;

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
                        mCurve[clip((int64_t)((red   - mBlack) * mScaleMul[0]))],
                        mCurve[clip((int64_t)((green - mBlack) * mScaleMul[1]))],
                        mCurve[clip((int64_t)((blue  - mBlack) * mScaleMul[2]))]
                    };
                }
        }
}

int compare(const void * a, const void * b)
{
    return (*(int*)a - *(int*)b);
}

int compared(const void * a, const void * b)
{
    return int(*(double*)a - *(double*)b);
}

void Debayer::ahd(RGB48Buffer *result)
{

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
        new float[mBayer->h * mBayer->w][3],
        new float[mBayer->h * mBayer->w][3]
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
                        cur = clip(mBayer->element(i + k, j + l));
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
                        green[0]->element(i + k, j + l) = clip(val);

                        // horizontal
                        val = weightedBayerAvg({ { j + l - 2, i + k },{ j + l - 1, i + k },{ j + l, i + k },{ j + l + 1, i + k },{ j + l + 2, i + k } }, filter) / 4;
                        val = clamp(val, weightedBayerAvg({ j + l - 1, i + k }), weightedBayerAvg({ j + l + 1, i + k }));
                        green[1]->element(i + k, j + l) = clip(val);
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
                        pixel[1] = clip(green[d]->element(i + k, j + l) * mScaleMul[1]);

                        uint8_t color = colorFromBayerPos(i + k, j + l, false);

                        if (color == 1)
                        {
                            uint8_t rowColor = colorFromBayerPos(i + k, j + l + 1, false);

                            // C = G + LP(C' - G'), where C is sought colour
                            // LP is in fact the average
                            val = pixel[1] + ((weightedBayerAvg({ j + l - 1, i + k }) - green[d]->element(i + k, j + l - 1)
                                             + weightedBayerAvg({ j + l + 1, i + k }) - green[d]->element(i + k, j + l + 1)) / 2);
                            
                            // logically, this should be pixel[rowColor], but our pixels are BGR, so this is inverted
                            pixel[2 - rowColor] = clip(val * mScaleMul[rowColor]);

                            val = pixel[1] + ((weightedBayerAvg({ j + l, i + k - 1 }) - green[d]->element(i + k - 1, j + l)
                                             + weightedBayerAvg({ j + l, i + k + 1 }) - green[d]->element(i + k + 1, j + l)) / 2);
                            pixel[rowColor] = clip(val * mScaleMul[2 - rowColor]);
                        }
                        else
                        {
                            // known colour: inverted (same as above)
                            pixel[2 - color] = clip(mBayer->element(i + k, j + l) * mScaleMul[color]);

                            // interpolate colour using greens diagonally
                            // this is not intuitive, but directly follows from the aforementioned equation
                            val = pixel[1] +
                                    ((weightedBayerAvg({ j + l - 1, i + k - 1 }) - green[d]->element(i + k - 1, j + l - 1)
                                    + weightedBayerAvg({ j + l + 1, i + k - 1 }) - green[d]->element(i + k + 1, j + l + 1)
                                    + weightedBayerAvg({ j + l - 1, i + k + 1 }) - green[d]->element(i + k - 1, j + l + 1)
                                    + weightedBayerAvg({ j + l + 1, i + k + 1 }) - green[d]->element(i + k + 1, j + l - 1)
                                     
                                     ) / 4);
                            pixel[color] = clip(val * mScaleMul[2 - color]);
                        }
                        rgb[d]->element(i + k, j + l) = pixel;
                        RGBConverter::rgb2Lab(pixel, Lab[d][(i + k) * mBayer->w + j + l]);
                    }
                }
            }
        }
    }

    // free some memory
    delete_safe(green[0]);
    delete_safe(green[1]);

    float *homo[2] = {
        new float[mBayer->h * mBayer->w],
        new float[mBayer->h * mBayer->w]
    };

    // build homogeneity maps using cielab metric
    for (int i = 0; i < mBayer->h; i++)
    {
        for (int j = 0; j < mBayer->w; j++)
        {
            int offset = i * mBayer->w + j;

            // luminance difference in 4 directions for 2 images
            float dl[2][4];

            // chrominance difference in 4 directions for 2 images
            float dc[2][4];

            for (int d = 0; d < 2; d++)
            {
                int idx = 0;
                for (int k = -1; k < 2; k += 2, idx++)
                {
                    int shiftA = k * mBayer->w;
                    if (offset + shiftA < 0 || offset + shiftA >= mBayer->h * mBayer->w)
                        continue;
                    dl[d][idx] = abs(Lab[d][offset][0] - Lab[d][offset + shiftA][0]);
                    dc[d][idx] = pow(Lab[d][offset][1] - Lab[d][offset + shiftA][1], 2) + pow(Lab[d][offset][2] - Lab[d][offset + shiftA][2], 2);
                }
                for (int l = -1; l < 2; l += 2, idx++)
                {
                    int shiftB = l;
                    if (offset + shiftB < 0 || offset + shiftB >= mBayer->h * mBayer->w)
                        continue;
                    dl[d][idx] = abs(Lab[d][offset][0] - Lab[d][offset + shiftB][0]);
                    dc[d][idx] = pow(Lab[d][offset][1] - Lab[d][offset + shiftB][1], 2) + pow(Lab[d][offset][2] - Lab[d][offset + shiftB][2], 2);
                }
            }

            // min { max { DIFF(x, neighbor) } } for all vertical/horizontal immediate neighbors from B(x, 1)
            // the luminance and chrominance deviations are defined as maximal deviation in any direction
            // we must choose minimal deviations to count homogenous pixels

            // VERSION A - proposed by Hirakawa & Parks, produces noticeable artifacts on cm_lighthouse.pgm
            //float epsL = min(max(dl[0][0], dl[0][1]), max(dl[1][2], dl[1][3]));
            float epsC = min(max(dc[0][0], dc[0][1]), max(dc[1][2], dc[1][3]));

            // VERSION B - extended, produces less artifacts on cm_lighthouse.pgm, but instead produces weak zipper on test_debayer.pgm
            float epsL = min(max(max(dl[0][0], dl[0][1]), max(dl[0][2], dl[0][3])), max(max(dl[1][0], dl[1][1]), max(dl[1][2], dl[1][3])));
            //float epsC = min(max(max(dc[0][0], dc[0][1]), max(dc[0][2], dc[0][3])), max(max(dc[1][0], dc[1][1]), max(dc[1][2], dc[1][3])));

            for (int d = 0; d < 2; d++)
            {
                int homo_val = 0;
                for (int j = 0; j < 4; j++)
                {
                    if (dl[d][j] <= epsL && dc[d][j] <= epsC)
                    {
                        homo_val++;
                    }
                }
                homo[d][offset] = homo_val;
            }
        }
    }

    int32_t *rgbDiff[4] = {
        new int32_t[mBayer->h * mBayer->w], // R - G
        new int32_t[mBayer->h * mBayer->w], // B - G
    };

    // select homogeneous pixels
    int homo_cur[2];
    for (int i = 0; i < mBayer->h; i++)
    {
        for (int j = 0; j < mBayer->w; j++)
        {
            int offset = i * mBayer->w + j;
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
                        int offset_l = k * mBayer->w + l;
                        for (int d = 0; d < 2; d++)
                            homo_cur[d] += homo[d][offset_l];
                    }

                if (homo_cur[0] > homo_cur[1])
                    result->element(i, j) = rgb[0]->element(i, j);
                else if (homo_cur[0] < homo_cur[1])
                    result->element(i, j) = rgb[1]->element(i, j);
                else
                    result->element(i, j) = (rgb[0]->element(i, j) + rgb[1]->element(i, j)) / 2;
            }
            rgbDiff[0][offset] = result->element(i, j).r() - result->element(i, j).g();
            rgbDiff[1][offset] = result->element(i, j).b() - result->element(i, j).g();
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
    const int size = (2 * radius + 1) * (2 * radius + 1);
    // median filter pass count, no difference except for running time observed between 1 and 2, more than 2 is redundant
    const int passes = 2;

    for (int p = 0; p < passes; p++)
        for (int i = radius; i < mBayer->h - radius; i++)
        {
            for (int j = radius; j < mBayer->w - radius; j++)
            {
                int32_t window[2][size];
                int offset = i * mBayer->w + j;
                for (int c = 0; c < 2; c++)
                {
                    int idx = 0;
                    for (int k = i - radius; k <= i + radius; k++)
                        for (int l = j - radius; l <= j + radius; l++)
                            window[c][idx++] = rgbDiff[c][k * mBayer->w + l];
                    qsort(window[c], size, sizeof(window[c][0]), compare);

                }

                uint32_t r = window[0][4] + result->element(i, j).g();
                uint32_t b = window[1][4] + result->element(i, j).g();
                uint32_t g = (r + b - window[0][4] - window[1][4]) / 2;
                result->element(i, j) = RGBColor48(
                    clip(r), 
                    clip(g),
                    clip(b)
                );

                rgbDiff[0][offset] = r - g;
                rgbDiff[1][offset] = b - g;
            }
        }

    deletearr_safe(rgbDiff[0]);
    deletearr_safe(rgbDiff[1]);
    deletearr_safe(rgbDiff[2]);
    deletearr_safe(rgbDiff[3]);
}

RGB48Buffer* Debayer::fourier()
{
    uint h = mBayer->h;
    uint w = mBayer->w;

    fftw_complex* in_r, *in_g, *in_b, *out_r, *out_g, *out_b;

    in_r = fftw_alloc_complex(h*w);
    in_g = fftw_alloc_complex(h*w);
    in_b = fftw_alloc_complex(h*w);

    out_r = fftw_alloc_complex(h*w);
    out_g = fftw_alloc_complex(h*w);
    out_b = fftw_alloc_complex(h*w);

    for (uint i = 0; i < h; i++)
        for (uint j = 0; j < w; j++)
        {
            uint offset = i * w + j;
            uint c = colorFromBayerPos(i, j, false);
            if (c == 0)
                in_r[offset][0] = mBayer->element(i, j);
            if (c == 1)
                in_g[offset][0] = mBayer->element(i, j);
            if (c == 2)
                in_b[offset][0] = mBayer->element(i, j);
            in_r[offset][1] = in_g[offset][1] = in_b[offset][1] = 0;
        }

    RGB48Buffer *out = new RGB48Buffer(h, w, false);
    G12Buffer *tmp2 = new G12Buffer(h, w, false);

    fftw_plan plan = fftw_plan_dft_2d(h, w, in_r, out_r, FFTW_FORWARD, 0);
    fftw_execute(plan);
    fftw_execute_dft(plan, in_g, out_g);
    fftw_execute_dft(plan, in_b, out_b);
    fftw_destroy_plan(plan);
    double coeff = 5.1 / 12;
    for (int i = 0; (uint)i < h; i++)
        for (int j = 0; (uint)j < w; j++)
        {
            int disty = abs(i - (int)h / 2);
            int distx = abs(j - (int)w / 2);
            //int dist = sqrt(pow(distx, 2) + pow(disty, 2));

            int mul = int(distx < w * coeff && disty < h * coeff);
            out_r[i*w + j][0] /= (h * w);
            out_r[i*w + j][1] /= (h * w);
            out_r[i*w + j][0] *= mul;
            out_r[i*w + j][1] *= mul;

            out_g[i*w + j][0] /= (h * w);
            out_g[i*w + j][1] /= (h * w);
            out_g[i*w + j][0] *= mul;
            out_g[i*w + j][1] *= mul;

            out_b[i*w + j][0] /= (h * w);
            out_b[i*w + j][1] /= (h * w);
            out_b[i*w + j][0] *= mul;
            out_b[i*w + j][1] *= mul;

            tmp2->element(i, j) = clip(sqrt(pow(out_r[i*w + j][0], 2) + pow(out_r[i*w + j][1], 2)) * 5000);
        }

    plan = fftw_plan_dft_2d(h, w, out_r, in_r, FFTW_BACKWARD, 0);
    fftw_execute(plan);
    fftw_execute_dft(plan, out_g, in_g);
    fftw_execute_dft(plan, out_b, in_b);
    fftw_destroy_plan(plan);

    double *rgbDiff[2] = {
        new double[h*w],
        new double[h*w]
    };

    double white = 0;
    double *val_r = new double[h * w],
           *val_g = new double[h * w],
           *val_b = new double[h * w];

    for (uint i = 0; i < h; i++)
        for (uint j = 0; j < w; j++)
        {
            uint offset = i * w + j;
            val_r[offset] = sqrt(pow(in_r[offset][0], 2) + pow(in_r[offset][1], 2));
            val_g[offset] = sqrt(pow(in_g[offset][0], 2) + pow(in_g[offset][1], 2));
            val_b[offset] = sqrt(pow(in_b[offset][0], 2) + pow(in_b[offset][1], 2));
            rgbDiff[0][offset] = val_r[offset] - val_g[offset];
            rgbDiff[1][offset] = val_b[offset] - val_g[offset];
        }

    // calc median maximum
    for (uint i = 2; i < h - 2; i++)
        for (uint j = 2; j < w - 2; j++)
        {
            int32_t window[25];

            uint idx = 0;
                for (uint k = i - 2; k <= i + 2; k++)
                    for (uint l = j - 2; l <= j + 2; l++)
                    {
                        uint offset2 = k * w + l;
                        window[idx++] = max(val_r[offset2], max(val_g[offset2], val_b[offset2]));
                    }
            qsort(window, 25, sizeof(window[0]), compare);
            if (white < window[12])
                white = window[12];

        }

    // apply median filter to rgb result
    // filter radius
    // radius of 1 gives nice results
    // radius of 2 gives less false color artifacts for some images, but the colors become somewhat degraded and blurred for other images
    const int radius = 1;
    // filter size - do not change
    const int size = (2 * radius + 1) * (2 * radius + 1);
    // median filter pass count, no difference except for running time observed between 1 and 2, more than 2 is redundant
    const int passes = 0;

    for (uint p = 0; p < passes; p++)
        for (uint i = radius; i < h - radius; i++)
        {
            for (uint j = radius; j < w - radius; j++)
            {
                double window[2][size];
                uint offset = i * w + j;
                for (uint c = 0; c < 2; c++)
                {
                    int idx = 0;
                    for (uint k = i - radius; k <= i + radius; k++)
                        for (uint l = j - radius; l <= j + radius; l++)
                            window[c][idx++] = rgbDiff[c][k * w + l];
                    qsort(window[c], size, sizeof(window[c][0]), compared);

                }

                double r = window[0][4] + val_g[offset];
                double b = window[1][4] + val_g[offset];
                double g = (r + b - window[0][4] - window[1][4]) / 2;
                
                val_r[offset] = r;
                val_g[offset] = g;
                val_b[offset] = b;

                rgbDiff[0][offset] = r - g;
                rgbDiff[1][offset] = b - g;
            }
        }


    double ampl = 1 * 255.0 / white;
    for (uint i = 0; i < h; i++)
        for (uint j = 0; j < w; j++)
        {
            int offset = i * w + j;
            out->element(i, j) = {
                clip(val_r[offset] * ampl),
                uint16_t(clip(val_g[offset] * ampl) / 2),
                clip(val_b[offset] * ampl),
            };
        }
    PPMLoader().save("four_out.pgm", out);
    PPMLoader().save("four_out_imag.pgm", tmp2);

    //deletearr_safe(in_r);
    //deletearr_safe(in_g);
    //deletearr_safe(in_b);

    //deletearr_safe(out_r);
    //deletearr_safe(out_g);
    //deletearr_safe(out_b);
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

void Debayer::toRGB48(Method method, RGB48Buffer *output)
{
    preprocess();

    switch (method)
    {
    case Nearest:
        nearest(output);
        break;
    case Bilinear:
        linear(output);
        break;
    default:
    case AHD:
        ahd(output);
        break;
    }
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

inline int32_t Debayer::weightedBayerAvg(const Vector2d32& coords)
{
    int32_t x = coords.x(),
            y = coords.y();
    if (x < 0)
        x = -x;

    if (y < 0)
        y = -y;

    if (x >= mBayer->w)
    {
        x = 2 * mBayer->w - coords.x() - 1;
    }

    if (y >= mBayer->h)
    {
        y = 2 * mBayer->h - y - 1;
    }

    if (x >= 0 && y >= 0 && x < mBayer->w && y < mBayer->h)
        return mBayer->element(y, x);
    else
        return 0;
}

inline int32_t Debayer::weightedBayerAvg(const vector<Vector2d32>& coords, const vector<int>& coeffs)
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
                result += weightedBayerAvg(coords[i]) * coeffs[i];
            else
                result += weightedBayerAvg(coords[i]);
        }
    }
    if (div == 0)
        return 0;
    else
        return result / div;
}

inline int32_t Debayer::clamp(int32_t x, int32_t a, int32_t b)
{
    if (a > b)
        SwapXY<int32_t>(a, b);
    if (x < a)
        return a;
    if (x > b)
        return b;
    return x;
}

inline uint8_t Debayer::colorFromBayerPos(uint i, uint j, bool rggb)
{
    if (rggb)   // r, g1, g2, b
        return   (j ^ (mBayerPos & 1)) & 1 | (((i ^ ((mBayerPos & 2) >> 1)) & 1) << 1);
    else        // r, g, b
        return (((j ^ (mBayerPos & 1)) & 1 | (((i ^ ((mBayerPos & 2) >> 1)) & 1) << 1)) + 1) >> 1;
}

inline uint16_t Debayer::clip(int32_t x)
{
    if (x < 0)
        return 0;
    if (x > mMaximum)
        return mMaximum;

    return (uint16_t)x;
}
