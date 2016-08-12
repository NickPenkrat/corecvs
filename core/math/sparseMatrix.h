#ifndef SPARSEMATRIX
#define SPARSEMATRIX

#include <vector>
#include <map>
#include <iostream>

#include "matrix.h"
#include "vector.h"

#ifdef WITH_MKL
#include <mkl.h>
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
    Vector dtrsv(Vector &v, bool upper = true, bool notrans = true);

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

    std::pair<bool, SparseMatrix> incompleteCholseky();

    void print(std::ostream& out = std::cout) const;
    friend std::ostream& operator<< (std::ostream &out, const SparseMatrix &sm);

    int h, w;
    Vector dtrsv_un(Vector &v); // Ux=b
    Vector dtrsv_ut(Vector &v); // xU=b
    Vector dtrsv_ln(Vector &v); // Lx=b
    Vector dtrsv_lt(Vector &v); // xL=b
private:
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
