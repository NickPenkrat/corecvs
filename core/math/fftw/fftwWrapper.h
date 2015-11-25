/**
* FFTW wrapper definitions
*
* \author  pavel.vasilev
* \date    Nov 23, 2015
*/

#ifndef FFTWWRAPPER_H_
#define FFTWWRAPPER_H_

#include "global.h"

#include <vector>
#include <cstdint>

#include "abstractBuffer.h"

struct fftw_plan_s;
typedef double fftw_complex[2];

namespace corecvs
{

class FFTW
{

private:
    std::vector<int> mDimensions;
    fftw_plan_s* mPlan = nullptr;
    AbstractBuffer<int16_t, int32_t> b;

    void doTransform(int h, int w, fftw_complex *input, fftw_complex *output, int sign);

public:

    enum Direction
    {
        Forward = 1,
        Backward = -1
    };

    FFTW();

    ~FFTW();

    /**
    * One-dimensional Fast Fourier Transform on complex data yielding complex data.
    *
    * \param   [in]  input     Input data.
    * \param   [out] output    Transform result.
    *
    */
    void transform(int size, fftw_complex *input, fftw_complex *output, int direction);

    /**
    * One-dimensional forward Fast Fourier Transform on real data yielding complex data.
    *
    * \param   [in]  input     Input data.
    * \param   [out] output    Transform result.
    *
    */
    void transformForward(int size, double* input, fftw_complex *output);

    /**
    * One-dimensional forward Fast Fourier Transform on complex data yielding complex data.
    *
    * \param   [in]  input     Input data.
    * \param   [out] output    Transform result.
    *
    */
    void transformForward(int size, fftw_complex *input, fftw_complex *output);

    /**
    * One-dimensional forward Fast Fourier Transform on real data yielding complex data.
    *
    * \param   [in]  input     Input data.
    * \param   [out] output    Transform result.
    *
    */
    void transformForward(std::vector<double>& input, fftw_complex *output);

    /**
    * One-dimensional backward Fast Fourier Transform.
    *
    * \param   [in]  input     Input data.
    * \param   [out] output    Transform result.
    *
    */
    void transformBackward(int size, fftw_complex *input, fftw_complex *output);

    /**
    * Two-dimensional forward Fast Fourier Transform on real data yielding complex data.
    *
    * \param   [in]  input     Input data.
    * \param   [out] output    Transform result.
    *
    */
    void transformForward2D(AbstractBuffer<uint16_t, int32_t> *input, fftw_complex *output);

    /**
    * Two-dimensional forward Fast Fourier Transform on complex data yielding complex data.
    *
    * \param   [in]  input     Input data.
    * \param   [out] output    Transform result.
    *
    */
    void transform2D(int sizeX, int sizeY, fftw_complex *input, fftw_complex *output, int direction);

};

}
#endif