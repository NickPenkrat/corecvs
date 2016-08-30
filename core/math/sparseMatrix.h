#ifndef SPARSEMATRIX
#define SPARSEMATRIX

#include <vector>
#include <map>
#include <iostream>
#include <memory>
#include <atomic>

#include "matrix.h"
#include "vector.h"

#ifdef WITH_MKL
#include <mkl.h>
#endif

#ifdef WITH_CUSPARSE
#include "cuda_runtime_api.h"
#include "cusparse.h"
#include "cuda.h"
#endif

namespace corecvs
{

/**
* Here we store sparse matrix in 3-array compressed row storage
* http://netlib.org/linalg/html_templates/node91.html
*
* The only difference is that we prefer zero-based indicies
* Note that current implementation may be highly inefficient, but
* it proposed for using in problems with < 1% non-zero entries, so
* O(nnz^2) should be significantly better O(n^2) even with poor
* implementation
*/
class SparseMatrix
{
public:
    //! \brief Creates sparse matrix from dense with specified threshold
    SparseMatrix(const Matrix &dense, double threshold = 0.0);
    //! \brief Creates sparse matrix from CRS data
    SparseMatrix(int h = 0, int w = 0, const std::vector<double> &values = {}, const std::vector<int> &columns = {}, const std::vector<int> &rowPointers = {0});
    //! \brief Creates sparse matrix from {point, value} data
    SparseMatrix(int h, int w, const std::map<std::pair<int, int>, double> &data);
    //! \brief Creates dense submatrix and return column idx
    Matrix denseRows(int x1, int y1, int x2, int y2, std::vector<int> &colIdx);
    //! \brief Creates dense submatrix and return row idx
    Matrix denseCols(int x1, int y1, int x2, int y2, std::vector<int> &rowIdx);
    //! \brief Cast to dense matrix
    explicit operator Matrix() const;
    SparseMatrix(const SparseMatrix &src, int x1, int y1, int x2, int y2);
    SparseMatrix(const SparseMatrix &M) : h(M.h), w(M.w), values(M.values), columns(M.columns), rowPointers(M.rowPointers)
    {
    }
    SparseMatrix &operator=(const SparseMatrix &M)
    {
        if (this == &M)
            return *this;
#ifdef WITH_CUSPARSE
        gpuPromotion = nullptr;
#endif
        h = M.h;
        w = M.w;
        values = M.values;
        columns = M.columns;
        rowPointers = M.rowPointers;
        return *this;
    }
    corecvs::Matrix denseSubMatrix(int x1, int y1, int x2, int y2) const;
    void denseSubMatrix(int x1, int y1, int x2, int y2, double* output, int stride = -1) const;
    void checkCorrectness() const;
    int getUBIndex(int i, int j) const;
    int getIndex(int i, int j) const;
#ifdef WITH_MKL
    //! \brief Note: deletion of MKL's deletions is your problem
    explicit operator sparse_matrix_t() const;
    SparseMatrix(const sparse_matrix_t &mklSparse);
#endif
#if 0
    bool isValid(bool full = false) const;
    void sort();
#endif
    /**
     * \brief Accesses element.
     * Note that it does not return reference and
     * is not efficient (it may scan all non-zero entries
     * in row y before returning 0)
     */
    double a(int y, int x) const;
    double&a(int y, int x);
    int nnz() const;
    double fillin() const;
    void spyPlot() const;
    Vector dtrsv(const Vector &v, bool upper = true, bool notrans = true) const;

    friend SparseMatrix operator -(const SparseMatrix &a);
    friend SparseMatrix operator *(const double       &lhs, const SparseMatrix &rhs);
    friend SparseMatrix operator *(const SparseMatrix &lhs, const double       &rhs);
    friend SparseMatrix operator /(const SparseMatrix &lhs, const double       &rhs);

    friend SparseMatrix operator *(const SparseMatrix &lhs, const SparseMatrix &rhs);
    friend SparseMatrix operator +(const SparseMatrix &lhs, const SparseMatrix &rhs);
    friend SparseMatrix operator -(const SparseMatrix &lhs, const SparseMatrix &rhs);

    friend Vector       operator *(const SparseMatrix &lhs, const Vector       &rhs);
    friend Vector       operator *(const Vector       &lhs, const SparseMatrix &rhs);

    //! \brief Transposes matrix
    SparseMatrix t() const;
    SparseMatrix upper() const;
    SparseMatrix ata() const;
    bool linSolve(const Vector &rhs, Vector &res, bool symmetric = false, bool posDef = false) const;
    static bool LinSolve(const SparseMatrix &m, const Vector &rhs, Vector &res, bool symmetric = false, bool posDef = false);
    /*
     * Linear system solving with use of schur-complement structure (only with block-diagonal lower-right part)
     * Note that you shoul use it only when you are sure that lower (block-diagonal) part is well-conditioned
     */
    static bool LinSolveSchurComplementInv(const corecvs::SparseMatrix &A, const corecvs::Vector &B, const std::vector<int> &diagBlocks, corecvs::Vector &res, bool symmetric = false, bool posDef = false);
    static bool LinSolveSchurComplementOld(const corecvs::SparseMatrix &A, const corecvs::Vector &B, const std::vector<int> &diagBlocks, corecvs::Vector &res, bool symmetric = false, bool posDef = false);
    static bool LinSolveSchurComplementNew(const corecvs::SparseMatrix &A, const corecvs::Vector &B, const std::vector<int> &diagBlocks, corecvs::Vector &res, bool symmetric = false, bool posDef = false);
    static bool LinSolveSchurComplement(const corecvs::SparseMatrix &A, const corecvs::Vector &B, const std::vector<int> &diagBlocks, corecvs::Vector &res, bool symmetric = false, bool posDef = false, bool explicitInv = false);
    bool        linSolveSchurComplement(const corecvs::Vector &B, const std::vector<int> &diagBlocks, corecvs::Vector &res, bool symmetric = false, bool posDef = false);

    std::pair<bool, SparseMatrix> incompleteCholseky(bool allow_parallel = true);

    void print(std::ostream& out = std::cout) const;
    friend std::ostream& operator<< (std::ostream &out, const SparseMatrix &sm);

    int h, w;
    Vector dtrsv_un(const Vector &v) const;
    Vector dtrsv_ut(const Vector &v) const;
    Vector dtrsv_ln(const Vector &v) const;
    Vector dtrsv_lt(const Vector &v) const;

#ifdef WITH_CUSPARSE
    void promoteToGpu() const
    {
        std::cout << "Starting promotion" << std::endl;
        gpuPromotion = std::unique_ptr<GPU_promotion>(new GPU_promotion(*this));
        std::cout << "Promoted" << std::endl;
    }
    static const int SPMV_RETRY = 3;
#endif
private:
#ifdef WITH_CUSPARSE
    struct GPU_promotion
    {
        typedef std::unique_ptr<double, decltype(&cudaFree)> GpuDoublePtr;
        typedef std::unique_ptr<int, decltype(&cudaFree)> GpuIntPtr;
        GpuDoublePtr dev_values = GpuDoublePtr(nullptr, cudaFree);
        GpuIntPtr dev_columns = GpuIntPtr(nullptr, cudaFree), dev_rowPointers = GpuIntPtr(nullptr, cudaFree);
        std::atomic<int> total, cpu, gpu;

        GPU_promotion(const SparseMatrix &m) : total(0), cpu(0), gpu(0)
        {
            double *dv;
            std::cout << "Trying to allocate values  (" << m.nnz() * sizeof(double) / 1024.0 / 1024.0 << "Mb)" << std::endl;
            cudaMalloc(&(void*&)dv, m.nnz() * sizeof(double));
            dev_values = GpuDoublePtr(dv, cudaFree);

            int *dc;
            std::cout << "Trying to allocate columns (" << m.nnz() * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
            cudaMalloc(&(void*&)dc, m.nnz() * sizeof(int));
            dev_columns = GpuIntPtr(dc, cudaFree);

            int *drp;
            std::cout << "Trying to allocate rowPointers (" << (m.h + 1) * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
            cudaMalloc(&(void*&)drp, (m.h + 1) * sizeof(int));
            dev_rowPointers = GpuIntPtr(drp, cudaFree);

            std::cout << "Copying values  (" << m.nnz() * sizeof(double) / 1024.0 / 1024.0 << "Mb)" << std::endl;
            cudaMemcpy(dv, &m.values     [0], m.nnz() * sizeof(double), cudaMemcpyHostToDevice);
            std::cout << "Copying columns (" << m.nnz() * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
            cudaMemcpy(dc, &m.columns    [0], m.nnz() * sizeof(int)   , cudaMemcpyHostToDevice);
            std::cout << "Copying rowPointers (" << (m.h + 1) * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
            cudaMemcpy(drp,&m.rowPointers[0], (m.h+1) * sizeof(int)   , cudaMemcpyHostToDevice);

            auto err = cudaGetLastError();
            if (err != cudaSuccess)
            {
                fprintf(stderr, "Promotion failed: %s\n", cudaGetErrorString(err));
            }
        }
    };
    mutable std::unique_ptr<GPU_promotion> gpuPromotion = nullptr;
#endif
    void swapCoords(int &x1, int &y1, int &x2, int &y2) const;
    //! All non-zero entries of matrix
    std::vector<double> values;
    //! Column-indicies (of values)
    std::vector<int> columns;
    /**
     * Row start pointers; the last one equals to total number of NNZ
     * elements
     */
    std::vector<int> rowPointers;
};
SparseMatrix operator -(const SparseMatrix &a);
SparseMatrix operator *(const double       &lhs, const SparseMatrix &rhs);
SparseMatrix operator *(const SparseMatrix &lhs, const double       &rhs);
SparseMatrix operator /(const SparseMatrix &lhs, const double       &rhs);

SparseMatrix operator *(const SparseMatrix &lhs, const SparseMatrix &rhs);
SparseMatrix operator +(const SparseMatrix &lhs, const SparseMatrix &rhs);
SparseMatrix operator -(const SparseMatrix &lhs, const SparseMatrix &rhs);

Vector       operator *(const SparseMatrix &lhs, const Vector       &rhs);
Vector       operator *(const Vector       &lhs, const SparseMatrix &rhs);
std::ostream& operator<< (std::ostream &out, const SparseMatrix &sm);

}

#endif // SPARSEMATRIX
