/**
 * \file ppmLoader.cpp
 * \brief Add Comment Here
 *
 *  TODO : Remove doubling code in PPM and PPM6 loader ASAP
 *
 * \ingroup cppcorefiles
 * \date Jun 22, 2010
 * \author alexander
 */
#include <string>

#include "global.h"

#include "ppmLoader.h"

string PPMLoader::prefix1(".pgm");
string PPMLoader::prefix2(".ppm");

bool PPMLoader::acceptsFile(string name)
{
    return (
        name.compare(name.length() - prefix1.length(), prefix1.length(), prefix1) == 0 ||
        name.compare(name.length() - prefix2.length(), prefix2.length(), prefix2) == 0
        );
}

G12Buffer* PPMLoader::load(string name)
{
    return load(name, nullptr);
}

G12Buffer* PPMLoader::load(const string& name, MetaData *metadata)
{
    if (metadata != NULL) {
        DOTRACE(("Will load the file %s as PPM with metadata\n ", name.c_str()));
    }
    else {
        DOTRACE(("Will load the file %s as PPM (ignoring any available metadata)\n ", name.c_str()));
    }

    G12Buffer *toReturn = g12BufferCreateFromPGM(name, metadata);
    return toReturn;
}

G12Buffer* PPMLoader::g12BufferCreateFromPGM(const string& name, MetaData *meta)
{
    FILE      *fp = NULL;
    uint8_t   *charImage = NULL;
    G12Buffer *result = NULL;

    // PPM headers variable declaration
    unsigned long int i, j;
    unsigned long int h, w;
    uint8_t type;
    unsigned short int maxval;
    int shiftCount = 0;

    // open file for reading in binary mode
    fp = fopen(name.c_str(), "rb");

    if (fp == nullptr)
    {
        return nullptr;
    }

    if(!readHeader(fp, &h, &w, &maxval, &type, meta) || (type != 5 && type != 6))
    {
        fclose(fp);
        return nullptr;
    }

    bool calcWhite = false;
    int white = 0;
    // if no metadata is present, create some
    // if metadata is null, don't
    if (meta != nullptr)
    {
        // create an alias to metadata
        MetaData &metadata = *meta;
        // get significant bit count
        if (metadata["bits"].empty())
        {
            metadata["bits"].push_back(1);
        }
        while (maxval >> int(metadata["bits"][0]))
        {
            metadata["bits"][0]++;
        }
        if (metadata["white"].empty())
            calcWhite = true;
    }

    result = new G12Buffer(h, w, false);

    // image size in bytes
    uint64_t size = (maxval < 0x100 ? 1 : 2) * w * h;

    // for reading we don't need to account for possible system byte orders, so just use a 8bit buffer
    charImage = new uint8_t[size];

    if (fread(charImage, 1, size, fp) == 0)
    {
        CORE_ASSERT_FAIL("fread() call failed");
        goto done;
    }

    if (maxval <= 0xff)
    {
        // 1-byte case
        for (i = 0; i < h; i++)
            for (j = 0; j < w; j++)
            {
                result->element(i, j) = (charImage[i * w + j]);

                if (calcWhite)
                    if (result->element(i, j) > white)
                        white = result->element(i, j);
            }
    }
    else
    {
        // 2-byte case
        // here we need to calculate shift to compress data into a 12bit buffer
        for (shiftCount = 0; (maxval >> shiftCount) > G12Buffer::BUFFER_MAX_VALUE; shiftCount++);

        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w * 2; j += 2)
            {
                int offset = i * w * 2 + j;
                result->element(i, j / 2) = ((charImage[offset + 0]) << 8 |
                    (charImage[offset + 1])) >> shiftCount;

                CORE_ASSERT_FALSE((result->element(i, j / 2) >= (1 << G12Buffer::BUFFER_BITS)), "Internal error in image loader\n");

                if (calcWhite)
                    if (result->element(i, j / 2) > white)
                        white = result->element(i, j / 2);
            }
        }

    }

    if (calcWhite)
        meta->at("white").push_back(white);

done:
    if (fp != NULL)
        fclose(fp);
    if (charImage != NULL)
        deletearr_safe(charImage);
    return result;
}

std::unique_ptr<char[]> PPMLoader::nextLine(FILE *fp, int sz, MetaData *metadata)
{
    std::unique_ptr<char[]> buf = new char[sz];
    while (fread(buf, 1, 1, fp))
    {

        if (buf[0] != '#' && buf[0] != '\n' && buf[0] != '\r')
        {
            fseek(fp, -1, SEEK_CUR);
            if (sz > 0 && fgets(buf, sz, fp) == NULL)
            {
                printf("fgets() call failed %s:%d\n", __FILE__, __LINE__);
            }
            return buf;
        }
        else
        {
            fgets(buf, sz, fp);

            // try to read metadata
            char param[256];
            int n = 0;

            // read param name
            if (metadata != nullptr && sscanf(buf, " @meta %s\t@values %d\t", param, &n) == 2)
            {
                char* numbers = strrchr(buf, '\t') + 1;
                MetaValue values;
                // read n param values
                for (int i = 0; i < n; i++)
                {
                    double v = 0;
                    sscanf(numbers, "%lf", &v);
                    numbers = strchr(numbers, ' ') + 1;
                    values.push_back(v);
                }
                metadata->insert(MetaPair(param, values));
            }
            memset(buf, 0, sz);
        }
    }

    return nullptr;
}

bool PPMLoader::readHeader(FILE *fp, unsigned long int *h, unsigned long int *w, uint16_t *maxval, uint8_t *type, MetaData* metadata)
{
    // skip comments and read next line
    std::unique_ptr<char[]> header = nextLine(fp, 255, metadata);

    // check PPM type (currently only supports 5 or 6)
    if ((header[0] != 'P') || (header[1] < '5') || (header[1] > '6'))
    {
        //printf("Image is not a supported PPM\n");
        return false;
    }

    // type contains a TYPE uint8_t, in our case 5 or 6
    *type = header[1] - '0';

    // get dimensions
    header = nextLine(fp, 255, metadata);

    // parse dimensions
    if (sscanf(header, "%lu%lu", w, h) != 2)
    {
        // try to parse dimensions in Photoshop-like format (when a newline is used instead of whitespace or tabulation)
        if (sscanf(header, "%lu", w) != 1)
        {
            //printf("Image dimensions could not be read from line %s\n", header);
            return false;
        }
        else
        {
            // first dimension has been read, try to read the second
            header = nextLine(fp, 255, metadata);
            if (sscanf(header, "%lu", h) != 1) // duplicate code can be gotten rid of with a goto (not sure it's worth doing)
            {
                //printf("Image dimensions could not be read from line %s\n", header);
                return false;
            }
        }
    }

    // get colour depth (metric?)
    header = nextLine(fp, 255, metadata);

    if (sscanf(header, "%hu", maxval) != 1)
    {
        //printf("Image metric could not be read form line %s\n", header);
        return false;
    }

    // we assume that no comments exist after the colour depth header line to avoid misinterpretation of '#' first data value

    DOTRACE(("Image is P6 PPM [%lu %lu] max=%u\n", *h, *w, *maxval));
    return true;
}

bool PPMLoader::writeHeader(FILE *fp, unsigned long int h, unsigned long int w, uint8_t type, uint16_t maxval, MetaData* meta)
{
    if (!fp || !h || !w || type < 5 || type > 6)
        return false;

    MetaData &metadata = *meta;

    fprintf(fp, "P%u\n", type);
    fprintf(fp, "############################################\n");
    fprintf(fp, "# This file is written by DeepView library.\n");

    // TODO: test this!
    if (meta)
        for (MetaData::iterator i = metadata.begin(); i != metadata.end(); i++)
        {
            fprintf(fp, "# @meta %s\t@values %i\t", i->first.c_str(), (int)i->second.size());
            for (int j = 0; j < i->second.size(); j++)
                fprintf(fp, "%f ", i->second[j]);
            fprintf(fp, "\n");
        }

    fprintf(fp, "############################################\n");
    fprintf(fp, "%lu %lu\n", w, h);
    fprintf(fp, "%hu\n", maxval);

    return true;
}

int PPMLoader::save(const string& name, G12Buffer *buffer, MetaData* metadata)
{
    int h = buffer->h;
    int w = buffer->w;

    if (buffer == NULL)
        return -1;

    FILE *fp = fopen(name.c_str(), "wb");

    if (fp == NULL)
    {
        CORE_ASSERT_FAIL("Image could not be written");
        return -1;
    }

    writeHeader(fp, h, w, 5, G12Buffer::BUFFER_MAX_VALUE, metadata);

    uint8_t *charImage = new uint8_t[2 * w * h];
    int i, j;
    for (i = 0; i < h; i++)
        for (j = 0; j < w; j++)
        {
            int offset = i * w + j;
            charImage[offset * 2] = (buffer->element(i, j) >> 8) & 0xFF;
            charImage[offset * 2 + 1] = buffer->element(i, j) & 0xFF;
        }
    fwrite(charImage, 2, h * w, fp);
    deletearr_safe(charImage);
    fclose(fp);
    return 0;
}

int PPMLoader::save(const string& name, RGB24Buffer *buffer, MetaData* metadata)
{
    int h = buffer->h;
    int w = buffer->w;

    if (buffer == NULL)
        return -1;

    FILE *fp = fopen(name.c_str(), "wb");

    if (fp == NULL)
    {
        CORE_ASSERT_FAIL("Image could not be written");
        return -1;
    }

    writeHeader(fp, h, w, 6, 0xff, metadata);

    uint8_t *charImage = new uint8_t[3 * w * h];

    for (int i = 0; i < buffer->h; i++)
        for (int j = 0; j < buffer->w; j++)
        {
            uint8_t elemval[3] = {
                buffer->element(i, j).r(),
                buffer->element(i, j).g(),
                buffer->element(i, j).b()
            };

            for (int offset = i*buffer->w + j, k = 0; k < 3; k++)
                charImage[offset * 3 + k] = elemval[k];
        }

    fwrite(charImage, 3, h * w, fp);

    deletearr_safe(charImage);
    fclose(fp);
    return 0;
}

int PPMLoader::save(const string& name, RGB48Buffer *buffer, MetaData* metadata)
{
    int h = buffer->h;
    int w = buffer->w;

    if (buffer == NULL)
        return -1;

    FILE *fp = fopen(name.c_str(), "wb");

    if (fp == NULL)
    {
        CORE_ASSERT_FAIL("Image could not be written");
        return -1;
    }

    int maxval = 0;
    for (int i = 0; i < buffer->h; i++)
        for (int j = 0; j < buffer->w; j++)
        {
            if (buffer->element(i, j).r() > maxval)
                maxval = buffer->element(i, j).r();
            if (buffer->element(i, j).g() > maxval)
                maxval = buffer->element(i, j).g();
            if (buffer->element(i, j).b() > maxval)
                maxval = buffer->element(i, j).b();
        }

    int bits = 1;
    while (maxval >> bits)
        bits++;

    int bytes = (bits + 7) / 8;

    writeHeader(fp, h, w, 6, (1 << bits) - 1, metadata);

    uint8_t *charImage = new uint8_t[3 * bytes * w * h];

    for (int i = 0; i < buffer->h; i++)
        for (int j = 0; j < buffer->w; j++)
        {
            uint16_t elemval[3] = {
                buffer->element(i, j).r(),
                buffer->element(i, j).g(),
                buffer->element(i, j).b()
            };

            if (bytes == 2)
                for (int offset = i*buffer->w + j, k = 0; k < 3; k++)
                {
                    charImage[(offset * 3 + k) * 2] = (elemval[k] & 0xff00) >> 8;
                    charImage[(offset * 3 + k) * 2 + 1] = (elemval[k] & 0xff);
                }
            else
                for (int offset = i*buffer->w + j, k = 0; k < 3; k++)
                {
                    charImage[offset * 3 + k] = (elemval[k] & 0xff);
                }
        }

    fwrite(charImage, 3, bytes * h * w, fp);

    deletearr_safe(charImage);
    fclose(fp);
    return 0;
}

// TODO: get rid of G16-specific methods
int PPMLoader::saveG16(const string& name, G12Buffer *buffer)
{
    int h = buffer->h;
    int w = buffer->w;

    if (buffer == NULL)
        return -1;

    FILE *fp;
    fp = fopen(name.c_str(), "wb");

    if (fp == NULL)
    {
        return -1;
    }

    fprintf(fp, "P5\n");
    fprintf(fp, "############################################\n");
    fprintf(fp, "# This file is written by DeepView library.\n");
    fprintf(fp, "# \n");
    fprintf(fp, "# The original source buffer had 16 bit.\n");

    /// \todo TODO: Add some metadata saving

    fprintf(fp, "############################################\n");
    fprintf(fp, "%d %d\n", w, h);
    fprintf(fp, "%d\n", 0xFFFF);

    uint8_t *charImage = new uint8_t[2 * w * h];
    int i, j;
    for (i = 0; i < h; i++)
    {
        for (j = 0; j < w; j++)
        {
            int offset = i * w + j;
            charImage[offset * 2] = (buffer->element(i, j) >> 8) & 0xFF;
            charImage[offset * 2 + 1] = buffer->element(i, j) & 0xFF;
        }
    }
    fwrite(charImage, 2, h * w, fp);
    deletearr_safe(charImage);
    fclose(fp);
    return 0;
}

/* Temporary 16bit load */

G12Buffer* PPMLoader::g16BufferCreateFromPPM(const string& name, MetaData* metadata)
{
    FILE      *fp = NULL;
    uint8_t   *charImage = NULL;
    G12Buffer *toReturn = NULL;


    // PGM Headers Variable Declaration
    unsigned long int i, j;
    unsigned long int h, w;
    uint8_t type;
    unsigned short int maxval;
    int shiftCount = 0;

    //Open file for Reading in Binary Mode
    fp = fopen(name.c_str(), "rb");

    if (fp == NULL)
    {
        printf("Image %s does not exist \n", name.c_str());
        return NULL;
    }

    if (!readHeader(fp, &h, &w, &maxval, &type, metadata))
    {
        return NULL;
    }

    if (maxval <= 255)
    {
        charImage = new uint8_t[w * h];
        if (fread(charImage, sizeof(uint8_t), w * h, fp) == 0)
        {
            printf("fread() call failed %s():%d\n", __FILE__, __LINE__);
            goto done;
        }

        toReturn = new G12Buffer(h, w, false);

        for (i = 0; i < h; i++)
            for (j = 0; j < w; j++)
                toReturn->element(i, j) = (charImage[i * w + j]) << 8;

    }
    else
    {
        for (shiftCount = 0; (maxval >> shiftCount) >(1 << 16); shiftCount++);

        charImage = new uint8_t[2 * w * h];
        if (fread(charImage, sizeof(uint8_t), (w * h) * 2, fp) == 0)
        {
            printf("fread() call failed %s:%d\n", __FILE__, __LINE__);
            goto done;
        }

        toReturn = new G12Buffer(h, w, false);

        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                int offset = i * w + j;
                toReturn->element(i, j) = ((charImage[offset * 2]) << 8 |
                    (charImage[offset * 2 + 1])) >> shiftCount;

                CORE_ASSERT_FALSE((toReturn->element(i, j) >= G12Buffer::BUFFER_MAX_VALUE), "Internal error in image loader\n");
            }
        }
    }

done:
    if (fp != NULL)
        fclose(fp);
    if (charImage != NULL)
        deletearr_safe(charImage);
    return toReturn;
}
