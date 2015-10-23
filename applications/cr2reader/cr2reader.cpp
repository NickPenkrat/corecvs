#include "cr2reader.h"
// the define fixes winsock2 conflicts for inclusion
#define NO_WINSOCK
#include <libraw.h>
#undef NO_WINSOCK

using namespace corecvs;

CR2Reader::CR2Reader()
{
    reader = new LibRaw;
    reader->imgdata.params.user_qual = 3; // default quality
    reader->imgdata.params.output_bps = 12; // default bit depth
}

int CR2Reader::open(const string& filename)
{
    int result = reader->open_file(filename.c_str());
    result |= reader->unpack();
    reader->imgdata.params.use_auto_wb = 0;
    reader->imgdata.params.use_camera_wb = 1;

    int maxval = 0;
    shift = 0;

    for (int i = 0; i < reader->imgdata.sizes.raw_height*reader->imgdata.sizes.raw_width; i++)
    {
        if (reader->imgdata.rawdata.raw_image[i] > maxval)
            maxval = reader->imgdata.rawdata.raw_image[i];
    }

    while (maxval >> shift > (1 << reader->imgdata.params.output_bps) - 1)
        shift++;

    return result;
}

void CR2Reader::setBPP(uint depth)
{
    reader->imgdata.params.output_bps = depth;
}

void CR2Reader::setQuality(uint quality)
{
    reader->imgdata.params.user_qual = quality;
}

void CR2Reader::histUpdate(int i, int j, uint16_t val)
{

    bool is_red = !(j & 1) && !(i & 1);
    bool is_blue = (j & 1) && (i & 1);
    bool is_green1 = !is_red && !is_blue && !(i & 1);
    bool is_green2 = !is_red && !is_blue && (i & 1);
    if (!hist)
    {
        hist = new uint64_t*[3];
        for (int k = 0; k < 3; k++)
        {
            hist[k] = new uint64_t[1 << 16];
            for (int l = 0; l < 1 << 16; l++)
                hist[k][l] = 0;
        }
    }
    int colour = is_green1 + is_green2 + is_blue * 2;
    hist[colour][val]++;

}

G12Buffer *CR2Reader::getBayer(bool shifted)
{
    if (hist)
    {
        for (int i = 0; i < 3; i++)
            delete[] hist[i];
        delete[] hist;
    }

    G12Buffer *result = new G12Buffer(reader->imgdata.sizes.height, reader->imgdata.sizes.width);
    int locShift = shifted * shift;
    for (int i = 0; i < reader->imgdata.sizes.height; i++)
        for (int j = 0; j < reader->imgdata.sizes.width; j++)
        {
            int offset = (i + reader->imgdata.sizes.top_margin)*reader->imgdata.sizes.raw_width + (j + reader->imgdata.sizes.left_margin);
            result->element(i, j) = reader->imgdata.rawdata.raw_image[offset] >> locShift;
            histUpdate(i, j, reader->imgdata.rawdata.raw_image[offset] >> locShift);
        }
    return result;
}

int CR2Reader::flipIndex(int row, int col)
{
    if (reader->imgdata.sizes.flip & 4) { row = row + col; col = row - col; row = row - col; };
    if (reader->imgdata.sizes.flip & 2) row = reader->imgdata.sizes.iheight - 1 - row;
    if (reader->imgdata.sizes.flip & 1) col = reader->imgdata.sizes.iwidth - 1 - col;
    return row * reader->imgdata.sizes.iwidth + col;
}

int isBigEndian()
{
    union
    {
        uint32_t i;
        char c[4];
    } bint = { 0x01020304 };

    return bint.c[0] == 1;
}

int CR2Reader::writePPM(const string& filename, bool fullcolour)
{
    // TODO: implement this in PPMLoader!
    int h = reader->imgdata.sizes.height;
    int w = reader->imgdata.sizes.width;

    if (reader == NULL || reader->imgdata.params.output_bps > 16 || reader->imgdata.params.output_bps < 1 || !h || !w)
        return -1;

    FILE *fp;
    fp = fopen(filename.c_str(), "wb");

    if (fp == NULL)
    {
        printf("Image %s could not be written \n", filename);
        return -1;
    }

    fprintf(fp, "P6\n");
    fprintf(fp, "############################################\n");
    fprintf(fp, "# The original file was a CR2.\n");
    fprintf(fp, "# Bit depth: %i bits per channel.\n", reader->imgdata.params.output_bps);

    /// \todo TODO: Add some metadata saving

    fprintf(fp, "############################################\n");
    fprintf(fp, "%d %d\n", w, h);

    cout << "Writing " << reader->imgdata.params.output_bps * 3 << " bits per pixel with quality level " << reader->imgdata.params.user_qual << "..." << std::endl;

    // calculate bitwise shift for our custom colour depth
    if (reader->imgdata.params.output_bps <= 8 || fullcolour)
        shift = 16 - reader->imgdata.params.output_bps;
    else
        shift = 8;

    fprintf(fp, "%d\n", (1 << (16 - shift)) - 1);

    uint8_t *img8 = new uint8_t[3 * 2 * w * h];

    // 16bit buffer is just the 8bit buffer with a different pointer
    ushort *img16 = (ushort*)img8;

    // to account for different image matrix configurations, we use libraw-calculated steps
    int soff = flipIndex(0, 0);
    int cstep = flipIndex(0, 1) - soff;
    int rstep = flipIndex(1, 0) - flipIndex(0, reader->imgdata.sizes.width);
    int colors = reader->imgdata.idata.colors;
    int width = reader->imgdata.sizes.width;
    int height = reader->imgdata.sizes.height;
    int t_white = 0x2000;

    // white level percentile (currently unused as we have no histogram whatsoever)
    //perc = width * height * reader->imgdata.params.auto_bright_thr;

    // calculate gamma correction
    reader->gamma_curve(reader->imgdata.params.gamm[0], reader->imgdata.params.gamm[1], 2, (t_white << 3) / reader->imgdata.params.bright);
    reader->imgdata.sizes.iheight = height;
    reader->imgdata.sizes.iwidth = width;

    int bytesperpixel = fullcolour ? ((reader->imgdata.params.output_bps + 7) / 8) : 1;

    // the following code was taken from libraw
    for (int row = 0; row < height; row++, soff += rstep)
    {
        for (int col = 0; col < width; col++, soff += cstep)
            if (bytesperpixel == 1)
                for (int c = 0; c < colors; c++)
                    img8[col*colors + c] = reader->imgdata.color.curve[reader->imgdata.image[soff][c]] >> shift;
            else for (int c = 0; c < colors; c++)
                img16[col*colors + c] = reader->imgdata.color.curve[reader->imgdata.image[soff][c]] >> shift;
        if (bytesperpixel == 2 && !isBigEndian())
            swab((char*)img16, (char*)img16, width*colors * 2);
        fwrite(img8, colors * bytesperpixel, width, fp);
    }

    delete[] img8;
    fclose(fp);
    return 0;
}

MetaData* CR2Reader::getMetadata()
{
    MetaData* meta = new MetaData;
    MetaData &metadata = *meta;

    uint16_t maxval = 0;

    for (int i = 0; i < reader->imgdata.sizes.raw_height; i++)
        for (int j = 0; j < reader->imgdata.sizes.raw_width; j++)
        {
            int offset = (i + reader->imgdata.sizes.top_margin)*reader->imgdata.sizes.raw_width + (j + reader->imgdata.sizes.left_margin);
            if (maxval < reader->imgdata.rawdata.raw_image[offset])
                maxval = reader->imgdata.rawdata.raw_image[offset];
        }

    int perc = 0, val = 0, total = 0, c = 0;
    int t_white = 1 << 16;
    perc = reader->imgdata.sizes.width * reader->imgdata.sizes.height * reader->imgdata.params.auto_bright_thr;

    for (t_white = c = 0; c < 3; c++)
    {
        for (val = 0x2000, total = 0; --val > 32; )
            if ((total += hist[c][val]) > perc) break;
        if (t_white < val) t_white = val;
    }

    double* pre_mul = new double[3];
    memcpy(pre_mul, reader->imgdata.color.pre_mul, 3 * sizeof(double));
    double* cam_mul = new double[3];
    memcpy(cam_mul, reader->imgdata.color.cam_mul, 3 * sizeof(double));
    double* gamm = new double[6];
    memcpy(gamm, reader->imgdata.params.gamm, 6 * sizeof(double));

    metadata["pre_mul"] = MetaValue(3, pre_mul);
    metadata["cam_mul"] = MetaValue(3, cam_mul);
    metadata["gamm"] = MetaValue(6, gamm);
    metadata["type"] = reader->imgdata.idata.filters;
    metadata["white"] = maxval;
    metadata["black"] = reader->imgdata.color.black >> shift;
    metadata["t_white"] = t_white;

    return meta;
}

int CR2Reader::processDCRaw(bool noScale)
{
    reader->imgdata.params.no_auto_scale = noScale;
    return reader->dcraw_process();
}

void CR2Reader::fakeBayer(G12Buffer *img)
{
    int orig_width = reader->imgdata.rawdata.sizes.raw_width;
    reader->imgdata.rawdata.sizes.raw_width = img->w;
    reader->imgdata.rawdata.sizes.width = img->w;
    reader->imgdata.rawdata.sizes.iwidth = img->w;
    reader->imgdata.rawdata.sizes.raw_height = img->h;
    reader->imgdata.rawdata.sizes.height = img->h;
    reader->imgdata.rawdata.sizes.iheight = img->h;
    reader->imgdata.rawdata.sizes.left_margin = 0;
    reader->imgdata.rawdata.sizes.top_margin = 0;

    for (int i = 0; i < img->h; i++)
        for (int j = 0; j < img->w; j++)
        {
            reader->imgdata.rawdata.raw_image[i*orig_width + j] = img->element(i, j) << 8;
        }

}

CR2Reader::~CR2Reader()
{
    for (int i = 0; i < 3; i++)
        delete[] hist[i];
    delete[] hist;
    delete reader;
}
