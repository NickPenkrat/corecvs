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
    SparseMatrix(int h, int w, const std::vector<double> &values, const std::vector<int> &columns, const std::vector<int> &rowPointers);
    //! \brief Creates sparse matrix from {point, value} data
    SparseMatrix(int h, int w, const std::map<std::pair<int, int>, double> &data);
    //! \brief Cast to dense matrix
    explicit operator Matrix() const;
#ifdef WITH_MKL
    explicit operator sparse_matrix_t();
#endif

    /**
     * \brief Accesses element.
     * Note that it does not return reference and
     * is not efficient (it may scan all non-zero entries
     * in row y before returning 0)
     */
    double a(int y, int x) const;

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

    void print(std::ostream& out = std::cout) const;
    friend std::ostream& operator<< (std::ostream &out, const SparseMatrix &sm);

private:
    int h, w;
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

}


#endif