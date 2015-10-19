#pragma once

#include "g12Buffer.h"
#include "rgbBufferT.h"
#include "metamap.h"

using namespace std;
using corecvs::G12Buffer;
class Debayer
{
private:
    G12Buffer* bayer;
    MetaData *metadata_ptr;
    int depth;
    // image metadata storage
    uint16_t* curve;
    double* scale_mul;
    uint16_t m_black;

    double* scale_colors(bool highlight = false);
    uint16_t* gamma_curve(int mode, int imax);

public:
    RGB48Buffer* out;

    Debayer(G12Buffer *bayer, int depth = 12, MetaData *data = nullptr);

    RGB48Buffer* linear();
    RGB48Buffer* nearest();

    int writePPM(string filename);
    void postprocess();


    ~Debayer();
};

