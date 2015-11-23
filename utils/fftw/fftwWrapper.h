/**
* FFTW wrapper definitions
*
* \author  pavel.vasilev
* \date    Nov 23, 2015
*/

#ifndef SIFTGPUWRAPPER_H
#define SIFTGPUWRAPPER_H

#include "global.h"

#include <fftw/fftw3.h>
#include <vector>
#include <cstdint>

#include "abstractBuffer.h"

namespace corecvs
{

class FFTW
{
private:
    std::vector<int> mDimensions;
    fftw_plan mPlan = nullptr;
    AbstractBuffer<int16_t, int32_t> b;

public:
    FFTW()
    {
    }

    ~FFTW()
    {
        if (mPlan)
            fftw_destroy_plan(mPlan);
        mDimensions.clear();
    }

    /**
    * One-dimensional forward Fast Fourier Transform on real data yielding complex data.
    *
    * \param   [in]  input     Input data.
    * \param   [out] output    Transform result.
    *
    */
    void transformForward(double* input, fftw_complex *output, int size)
    {
        if (mDimensions.size() == 1 && mDimensions[0] == size)
        {
            fftw_execute_dft_r2c(mPlan, input, output);
        }
        else
        {
            if (mPlan)
                fftw_destroy_plan(mPlan);

            mPlan = fftw_plan_dft_r2c_1d(size, input, output, 0);
            mDimensions.clear();
            mDimensions.push_back(size);
            fftw_execute(mPlan);
        }
    }

    void transform(fftw_complex *input, fftw_complex *output, int size, int direction)
    {
        if (mDimensions.size() == 1 && mDimensions[0] == size)
        {
            fftw_execute_dft(mPlan, input, output);
        }
        else
        {
            if (mPlan)
                fftw_destroy_plan(mPlan);

            mPlan = fftw_plan_dft_1d(size, input, output, FFTW_FORWARD, 0);
            mDimensions.clear();
            mDimensions.push_back(size);
            fftw_execute(mPlan);
        }
    }

    void transformForward(fftw_complex *input, fftw_complex *output, int size)
    {
        transform(input, output, size, FFTW_FORWARD);
    }

    void transformForward(std::vector<double> input, fftw_complex *output)
    {
        transformForward(input.data(), output, input.size());
    }

    void transformBackward(fftw_complex *input, fftw_complex *output, int size)
    {
        transform(input, output, size, FFTW_BACKWARD);
    }

    /**
    * Two-dimensional forward Fast Fourier Transform on real data yielding complex data.
    *
    * \param   [in]  input     Input data.
    * \param   [out] output    Transform result.
    *
    */
    template<typename ElementType>
    void transformForward2DT(AbstractBuffer<ElementType, int32_t> *input, fftw_complex *output)
    {
        double *indata = fftw_alloc_real(input->h * input->w);

        for (int i = 0; i < input->h; i++)
            for (int j = 0; j < input->w; j++)
            {
                indata[i * input->w + j] = input->element(i, j);
            }

        if (mDimensions.size() == 2 && mDimensions[0] == input->h && mDimensions[1] == input->w)
        {
            fftw_execute_dft_r2c(mPlan, indata, output);
        }
        else
        {
            if (mPlan)
                fftw_destroy_plan(mPlan);

            mPlan = fftw_plan_dft_r2c_2d(input->h, input->w, indata, output, 0);
            mDimensions.clear();
            mDimensions.push_back(input->h);
            mDimensions.push_back(input->w);
            fftw_execute(mPlan);
        }
    }

    void transformForward2D(AbstractBuffer<uint16_t, int32_t> *input, fftw_complex *output)
    {
        return transformForward2DT(input, output);
    }

    void transform2D(fftw_complex *input, fftw_complex *output, int sizeY, int sizeX, int sign)
    {
        /*if (mDimensions.size() == 2 && mDimensions[0] == sizeX && mDimensions[1] == sizeY)
        {
            fftw_execute_dft(mPlan, input, output);
        }
        else*/
        {
            if (mPlan)
                fftw_destroy_plan(mPlan);

            mPlan = fftw_plan_dft_2d(sizeX, sizeY, input, output, sign, 0);
            mDimensions.clear();
            mDimensions.push_back(sizeX);
            mDimensions.push_back(sizeY);
            fftw_execute(mPlan);
        }
    }

};

}
#endif