#ifndef EM_
#define EM_

#include <vector>
#include <ostream>

#include "matrix.h"

namespace corecvs
{
struct EM
{
    EM(const Matrix &A, int N, bool smooth = true, int maxIter = 1000, bool randomInit = false);

    double stepE();
    void stepM();
    void runEM(int maxIter = -1);
    void runKMeans();

    Matrix probabilities;

    std::vector<double> prior;
    std::vector<Vector> means;
    std::vector<Matrix> covariances;

    friend std::ostream& operator<<(std::ostream &os, const EM& em);

private:
    bool smooth;
    int N, M, K;
    const Matrix &A;
    int maxIter;

};
}

#endif
