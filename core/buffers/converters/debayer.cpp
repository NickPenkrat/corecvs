#include "debayer.h"

Debayer::Debayer(G12Buffer *bayer, MetaData *metadata)
{
    this->bayer = bayer;
    this->metadata_ptr = metadata;
}

Debayer* Debayer::FromFile(string filename)
{
    // this should work
    PPMLoader ldr;
    MetaData *metadata = new MetaData;
    G12Buffer* bayer = ldr.g12BufferCreateFromPGM(filename, metadata);
    return new Debayer(bayer, metadata);
}

// i have yet to understand what this actually does... 
// TODO: maybe just replace it with simple 3-colour loop?
int FC(int row, int col, int filters) { return (filters >> (((row << 1 & 14) | (col & 1)) << 1) & 3); }

G12Buffer** Debayer::nearest()
{
    G12Buffer *red = new G12Buffer(bayer->h, bayer->w, false);
    G12Buffer *green = new G12Buffer(bayer->h, bayer->w, false);
    G12Buffer *blue = new G12Buffer(bayer->h, bayer->w, false);

    for (int i = 0; i < bayer->h; i++)
        for (int j = 0; j < bayer->w; j++)
        {
            bool is_red = !(j & 1) && !(i & 1);
            bool is_blue = (j & 1) && (i & 1);
            bool is_green1 = !is_red && !is_blue && !(i & 1);
            bool is_green2 = !is_red && !is_blue && (i & 1);

            if (is_red)
            {
                red->element(i, j) = bayer->element(i, j);
                green->element(i, j) = bayer->element(i, j + 1);
                blue->element(i, j) = bayer->element(i + 1, j + 1);
            }
            if (is_green1)
            {
                red->element(i, j) = bayer->element(i, j - 1);
                green->element(i, j) = bayer->element(i, j);
                blue->element(i, j) = bayer->element(i + 1, j);
            }
            if (is_green2)
            {
                red->element(i, j) = bayer->element(i - 1, j);
                green->element(i, j) = bayer->element(i, j);
                blue->element(i, j) = bayer->element(i, j + 1);
            }
            if (is_blue)
            {
                red->element(i, j) = bayer->element(i - 1, j - 1);
                green->element(i, j) = bayer->element(i, j - 1);
                blue->element(i, j) = bayer->element(i, j);
            }
        }
    out = new G12Buffer*[3];
    out[0] = red; out[1] = green; out[2] = blue;
    return out;
}

G12Buffer** Debayer::linear()
{
    G12Buffer *red = new G12Buffer(bayer->h, bayer->w, false);
    G12Buffer *green = new G12Buffer(bayer->h, bayer->w, false);
    G12Buffer *blue = new G12Buffer(bayer->h, bayer->w, false);

    for (int i = 0; i < bayer->h; i++)
        for (int j = 0; j < bayer->w; j++)
        {

            bool is_red = !(j & 1) && !(i & 1);
            bool is_blue = (j & 1) && (i & 1);
            bool is_green1 = !is_red && !is_blue && !(i & 1);
            bool is_green2 = !is_red && !is_blue && (i & 1);

            if (is_red)
            {
                red->element(i, j) = bayer->element(i, j);

                if (!(i || j))
                {
                    green->element(i, j) = (bayer->element(i, j + 1) + bayer->element(i + 1, j)) / 2;
                    blue->element(i, j) = bayer->element(i + 1, j + 1);
                }
                else if (i == bayer->h - 1 && j == bayer->w - 1)
                {
                    green->element(i, j) = (bayer->element(i, j - 1) + bayer->element(i - 1, j)) / 2;
                    blue->element(i, j) = bayer->element(i - 1, j - 1);
                }
                else if (!i)
                {
                    green->element(i, j) = (bayer->element(i, j + 1) + bayer->element(i + 1, j) + bayer->element(i, j - 1)) / 3;
                    blue->element(i, j) = (bayer->element(i + 1, j + 1) + bayer->element(i + 1, j - 1)) / 2;
                }
                else if (!j)
                {
                    green->element(i, j) = (bayer->element(i, j + 1) + bayer->element(i + 1, j) + bayer->element(i - 1, j)) / 3;
                    blue->element(i, j) = (bayer->element(i + 1, j + 1) + bayer->element(i - 1, j + 1)) / 2;
                }
                else if (i == bayer->h - 1)
                {
                    green->element(i, j) = (bayer->element(i, j - 1) + bayer->element(i - 1, j) + bayer->element(i, j + 1)) / 3;
                    blue->element(i, j) = (bayer->element(i - 1, j + 1) + bayer->element(i - 1, j - 1)) / 2;
                }
                else if (j == bayer->w - 1)
                {
                    green->element(i, j) = (bayer->element(i, j - 1) + bayer->element(i + 1, j) + bayer->element(i - 1, j)) / 3;
                    blue->element(i, j) = (bayer->element(i + 1, j - 1) + bayer->element(i - 1, j - 1)) / 2;
                }
                else
                {
                    green->element(i, j) = (bayer->element(i, j + 1) + bayer->element(i, j - 1) + bayer->element(i + 1, j) + bayer->element(i - 1, j)) / 4;
                    blue->element(i, j) = (bayer->element(i + 1, j + 1) + bayer->element(i - 1, j + 1) + bayer->element(i + 1, j - 1) + bayer->element(i - 1, j - 1)) / 4;
                }

            }
            if (is_green1)
            {
                if (j != bayer->w - 1)
                    red->element(i, j) = (bayer->element(i, j - 1) + bayer->element(i, j + 1)) / 2;
                else
                    red->element(i, j) = bayer->element(i, j - 1);

                green->element(i, j) = bayer->element(i, j);

                if (i > 0)
                    blue->element(i, j) = (bayer->element(i + 1, j) + bayer->element(i - 1, j)) / 2;
                else
                    blue->element(i, j) = bayer->element(i + 1, j);
            }
            if (is_green2)
            {
                if (i != bayer->h - 1)
                    red->element(i, j) = (bayer->element(i + 1, j) + bayer->element(i - 1, j)) / 2;
                else
                    red->element(i, j) = bayer->element(i - 1, j);

                green->element(i, j) = bayer->element(i, j);

                if (j > 0)
                    blue->element(i, j) = (bayer->element(i, j - 1) + bayer->element(i, j + 1)) / 2;
                else
                    blue->element(i, j) = bayer->element(i, j - 1);

            }
            if (is_blue)
            {
                if (!(i || j))
                {
                    green->element(i, j) = (bayer->element(i, j + 1) + bayer->element(i + 1, j)) / 2;
                    red->element(i, j) = bayer->element(i + 1, j + 1);
                }
                else if (i == bayer->h - 1 && j == bayer->w - 1)
                {
                    green->element(i, j) = (bayer->element(i, j - 1) + bayer->element(i - 1, j)) / 2;
                    red->element(i, j) = bayer->element(i - 1, j - 1);
                }
                else if (!i)
                {
                    green->element(i, j) = (bayer->element(i, j + 1) + bayer->element(i + 1, j) + bayer->element(i, j - 1)) / 3;
                    red->element(i, j) = (bayer->element(i + 1, j + 1) + bayer->element(i + 1, j - 1)) / 2;
                }
                else if (!j)
                {
                    green->element(i, j) = (bayer->element(i, j + 1) + bayer->element(i + 1, j) + bayer->element(i - 1, j)) / 3;
                    red->element(i, j) = (bayer->element(i + 1, j + 1) + bayer->element(i - 1, j + 1)) / 2;
                }
                else if (i == bayer->h - 1)
                {
                    green->element(i, j) = (bayer->element(i, j - 1) + bayer->element(i - 1, j) + bayer->element(i, j + 1)) / 3;
                    red->element(i, j) = (bayer->element(i - 1, j + 1) + bayer->element(i - 1, j - 1)) / 2;
                }
                else if (j == bayer->w - 1)
                {
                    green->element(i, j) = (bayer->element(i, j - 1) + bayer->element(i + 1, j) + bayer->element(i - 1, j)) / 3;
                    red->element(i, j) = (bayer->element(i + 1, j - 1) + bayer->element(i - 1, j - 1)) / 2;
                }
                else
                {
                    green->element(i, j) = (bayer->element(i, j + 1) + bayer->element(i, j - 1) + bayer->element(i + 1, j) + bayer->element(i - 1, j)) / 4;
                    red->element(i, j) = (bayer->element(i + 1, j + 1) + bayer->element(i - 1, j + 1) + bayer->element(i + 1, j - 1) + bayer->element(i - 1, j - 1)) / 4;
                }

                blue->element(i, j) = bayer->element(i, j);
            }
        }
    out = new G12Buffer*[3];
    out[0] = red; out[1] = green; out[2] = blue;
    return out;
}

double* Debayer::scale_colors(bool highlight, MetaData *meta)
{
    double* scale_mul = new double[4];
    for (int i = 0; i < 4; i++)
        scale_mul[i] = 1;

    // alias for ease of use
    MetaData &metadata = *meta;

    // check if meta- and white balance data available
    if (meta == nullptr ||
        !metadata["cam_mul"] || !metadata["cam_mul"][0] || !metadata["cam_mul"][2] || !metadata["pre_mul"] || !metadata["pre_mul"][0])
        // TODO: maybe apply the gray world hypothesis instead of doing nothing
        return scale_mul; // if not, return vector of 1's

    unsigned c;
    double dmin, dmax;

    if (metadata["cam_mul"][0] != -1)
        for (int i = 0; i < 4; i++)
            metadata["pre_mul"][i] = metadata["cam_mul"][i]; // clarify: why are we doing this?

    if (!metadata["pre_mul"][1]) metadata["pre_mul"][1] = 1;
    if (!metadata["pre_mul"][3]) metadata["pre_mul"][3] = metadata["pre_mul"][1];

    metadata["white"][0] -= metadata["black"][0];
    for (dmin = DBL_MAX, dmax = c = 0; c < 4; c++)
    {
        if (dmin > metadata["pre_mul"][c])
            dmin = metadata["pre_mul"][c];
        if (dmax < metadata["pre_mul"][c])
            dmax = metadata["pre_mul"][c];
    }

    if (!highlight) dmax = dmin;
    double factor = 1;
    for (int c = 0; c < 4; c++)
        scale_mul[c] = (metadata["pre_mul"][c] /= dmax)*factor;
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

uint16_t* Debayer::gamma_curve(int mode, int imax, int depth, MetaData *meta)
{
    // this code is taken from LibRaw
    // TODO: rewrite?
    uint16_t *curve = new uint16_t[0x10000];
    for (int i = 0; i < 0x10000; i++)
        curve[i] = i;

    // alias
    MetaData &metadata = *meta;

    // if no gamm coefficients are present (or valid), return no transform
    if (meta == nullptr || !metadata["gamm"] || !(metadata["gamm"][0] || metadata["gamm"][1] || metadata["gamm"][2] || metadata["gamm"][3] || metadata["gamm"][4]))
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
        memcpy(metadata["gamm"], g, sizeof g);
        return curve;
    }
    for (i = 0; i < 0x10000; i++)
    {
        curve[i] = (1 << depth) - 1;
        if ((r = (double)i / imax) < 1)
            curve[i] = (1 << depth) * (mode
                ? (r < g[3] ? r*g[1] : (g[0] ? pow(r, g[0])*(1 + g[4]) - g[4] : log(r)*g[2] + 1))
                : (r < g[2] ? r / g[1] : (g[0] ? pow((r + g[4]) / (1 + g[4]), 1 / g[0]) : exp((r - 1) / g[2]))));
    }
    return curve;
}

uint16_t clip(int64_t x, int depth = 16)
{
    const uint16_t maximum = (1 << depth) - 1;
    if (x < 0)
        return 0;
    if (x > maximum || x >= (1 << 16))
        return maximum;
    return (uint16_t)x;
}

int Debayer::writePPM(string filename, int depth)
{
    FILE *fp;
    fp = fopen(filename.c_str(), "wb");

    if (fp == NULL)
    {
        printf("Image %s could not be written \n", filename.c_str());
        return -1;
    }

    fprintf(fp, "P6\n");
    fprintf(fp, "############################################\n");
    fprintf(fp, "# Custom demosaic test.\n");
    fprintf(fp, "# Writing %d bits per pixelasd.\n", depth);
    fprintf(fp, "############################################\n");
    fprintf(fp, "%d %d\n", bayer->w, bayer->h);
    fprintf(fp, "%d\n", (1 << depth) - 1);

    // alias for ease of use
    MetaData &metadata = *this->metadata_ptr;

    // always check for metadata contents
    int m_white = this->metadata_ptr != nullptr && metadata["white"] ? metadata["white"][0] : (1 << depth) - 1;
    int m_black = this->metadata_ptr != nullptr &&  metadata["black"] ? metadata["black"][0] : 0;
    int m_bits = this->metadata_ptr != nullptr &&  metadata["bits"] ? metadata["bits"][0] : depth;
    int shift = m_bits - depth;

    // double check the white level in case it looks suspicious
    if (!m_white || m_white < 0 || m_white == (1 << depth) - 1)
        for (int i = 0; i < bayer->h; i++)
            for (int j = 0; j < bayer->w; j++)
            {
                if ((bayer->element(i, j)) > m_white)
                    m_white = bayer->element(i, j);
            }

    int m_twhite = this->metadata_ptr != nullptr && metadata["t_white"] ? (int)metadata["t_white"][0] : m_white;

    int t_white = (shift < 0 ? m_twhite << -shift : m_twhite >> shift);

    //double scale_mul[4] = { 1, 1, 1, 1 };
    double* scale_mul = scale_colors(false, this->metadata_ptr);

    // t_white was chosen experimentally
    int bpp = (depth + 7) / 8;

    uint16_t* curve = gamma_curve(2, t_white, depth, this->metadata_ptr);

    uint8_t *out_bytes = new uint8_t[bayer->w*bayer->h * 3 * bpp];

    for (int i = 0; i < bayer->h; i++)
        for (int j = 0; j < bayer->w; j++)
            for (int offset = i*bayer->w + j, k = 0; k < 3; k++)
            {
                uint16_t elemval = clip((uint64_t)out[k]->element(i, j) - m_black);

                uint16_t val = curve[clip((shift >= 0 ? elemval >> shift : elemval << -shift) * scale_mul[k], depth)];

                if (bpp == 1)
                {
                    out_bytes[offset * 3 + k] = val;
                }
                else
                {
                    out_bytes[(offset * 3 + k) * 2] = (val & 0xff00) >> 8;
                    out_bytes[(offset * 3 + k) * 2 + 1] = val & 0xff;
                }
            }
    fwrite(out_bytes, bpp, bayer->w*bayer->h * 3, fp);
    fclose(fp);
    return 0;
}

Debayer::~Debayer()
{
}
