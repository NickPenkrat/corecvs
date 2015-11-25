#include "fftwWrapper.h"
#include "fftw/fftw3.h"

using namespace corecvs;

FFTW::FFTW() 
{
}

FFTW::~FFTW()
{
    if (mPlan)
        fftw_destroy_plan(mPlan);
    mDimensions.clear();
}

void FFTW::transformForward(int size, double* input, fftw_complex *output)
{
    if (!mDimensions.empty() && mDimensions[0] == size && mDimensions[1] == 0)
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
        mDimensions.push_back(0);
        fftw_execute(mPlan);
    }
}

void FFTW::transform(int size, fftw_complex *input, fftw_complex *output, int direction)
{
    doTransform(size, 0, input, output, direction);
}

void FFTW::transformForward(int size, fftw_complex *input, fftw_complex *output)
{
    transform(size, input, output, FFTW_FORWARD);
}

void FFTW::transformForward(std::vector<double>& input, fftw_complex *output)
{
    transformForward(input.size(), input.data(), output);
}

void FFTW::transformBackward(int size, fftw_complex *input, fftw_complex *output)
{
    transform(size, input, output, FFTW_BACKWARD);
}

void FFTW::transform2D(int sizeX, int sizeY, fftw_complex *input, fftw_complex *output, int direction)
{
    doTransform(sizeX, sizeY, input, output, direction);
}


void FFTW::doTransform(int sizeX, int sizeY, fftw_complex *input, fftw_complex *output, int sign)
{
    // TODO: clarify when it's possible to keep the old plan
    /*if (!mDimensions.empty() && mDimensions[0] == sizeX && mDimensions[1] == sizeY)
    {
        fftw_execute_dft(mPlan, input, output);
    }
    else*/
    {
        if (mPlan)
            fftw_destroy_plan(mPlan);
        
        if (sizeY > 0)
            mPlan = fftw_plan_dft_2d(sizeX, sizeY, input, output, sign, 0);
        else
            mPlan = fftw_plan_dft_1d(sizeX, input, output, sign, 0);

        mDimensions.clear();
        mDimensions.push_back(sizeX);
        mDimensions.push_back(sizeY);
        fftw_execute(mPlan);
    }
}

void FFTW::transformForward2D(AbstractBuffer<uint16_t, int32_t> *input, fftw_complex *output)
{
    // TODO: rewrite
    double *indata = fftw_alloc_real(input->h * input->w);

    for (int i = 0; i < input->h; i++)
        for (int j = 0; j < input->w; j++)
        {
            indata[i * input->w + j] = input->element(i, j);
        }

    if (!mDimensions.empty() && mDimensions[0] == input->h && mDimensions[1] == input->w)
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

    delete_safe(indata);
}