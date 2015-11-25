#include "errorMetrics.h"

using namespace corecvs;

double ErrorMetrics::mse(RGB48Buffer *img1, RGB48Buffer *img2)
{
    if (img1->w != img2->w || img1->h != img2->h)
        return -1;

    double err = 0;

    for (int i = 0; i < img1->h; i++)
    {
        for (int j = 0; j < img1->w; j++)
        {
            for (int c = 0; c < 2; c++)
                err += pow(img1->element(i, j)[c] - img2->element(i, j)[c], 2);
        }
    }

    return err / (img1->h * img1->w * 3);
}

double ErrorMetrics::mse(G12Buffer *img1, G12Buffer *img2)
{
    if (img1->w != img2->w || img1->h != img2->h)
        return -1;

    double err = 0;

    for (int i = 0; i < img1->h; i++)
    {
        for (int j = 0; j < img1->w; j++)
        {
            err += pow(img1->element(i, j) - img2->element(i, j), 2);
        }
    }

    return err / (img1->h * img1->w * 3);
}

double ErrorMetrics::psnr(RGB48Buffer *img1, RGB48Buffer *img2)
{
    // check image sizes
    if (img1->w != img2->w || img1->h != img2->h)
        return -1;

    double MSE = mse(img1, img2);

    if (MSE == 0)
        return 1;

    return 2 * log10((1 << 16) - 1) - log10(MSE / (3 * img1->h * img1->w));
}

double ErrorMetrics::psnr(G12Buffer *img1, G12Buffer *img2)
{
    // check image sizes
    if (img1->w != img2->w || img1->h != img2->h)
        return -1;

    double MSE = mse(img1, img2);

    if (MSE == 0)
        return 1;

    return 2 * log10((1 << 12) - 1) - log10(MSE / (3 * img1->h * img1->w));
}

double ErrorMetrics::rmsd(RGB48Buffer *img1, RGB48Buffer *img2)
{
    if (img1->w != img2->w || img1->h != img2->h)
        return -1;

    double MSE = mse(img1, img2);

    return sqrt(MSE);
}

double ErrorMetrics::rmsd(G12Buffer *img1, G12Buffer *img2)
{
    if (img1->w != img2->w || img1->h != img2->h)
        return -1;

    double MSE = mse(img1, img2);

    return sqrt(MSE);
}