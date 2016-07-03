#include "sparseMatrix.h"
#include "cblasLapackeWrapper.h"

#include <chrono>

#ifdef WITH_TBB
#include <tbb/tbb.h>
#endif

using namespace corecvs;

void corecvs::SparseMatrix::spyPlot() const
{
    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            double v = a(i, j);
            std::cout << (v == 0.0 ? " " : "*");
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

int corecvs::SparseMatrix::nnz() const
{
    return values.size();
}

double corecvs::SparseMatrix::fillin() const
{
    return double(nnz()) / w / h;
}

corecvs::SparseMatrix::SparseMatrix(const SparseMatrix &src, int x1, int y1, int x2, int y2)
{
    x1 = std::max(0, std::min(src.w, x1));
    x2 = std::max(0, std::min(src.w, x2));
    y1 = std::max(0, std::min(src.h, y1));
    y2 = std::max(0, std::min(src.h, y2));
    if (x2 < x1)
        std::swap(x1, x2);
    if (y2 < y1)
        std::swap(y1, y2);

    h = y2 - y1;
    w = x2 - x1;
    CORE_ASSERT_TRUE_S(h >= 0 && w >= 0);
    rowPointers.resize(h + 1);
    for (int i = y1; i < y2; ++i)
    {
        for (int j = src.rowPointers[i]; j < src.rowPointers[i + 1]; ++j)
        {
            if (src.columns[j] >= x2)
                break;
            if (src.columns[j] <  x1)
                continue;
            columns.push_back(src.columns[j] - x1);
            values.push_back(src.values[j]);
        }
        rowPointers[i - y1 + 1] = (int)values.size();
    }
}

corecvs::Matrix corecvs::SparseMatrix::denseSubMatrix(int x1, int y1, int x2, int y2) const
{
    x1 = std::max(0, std::min(w, x1));
    x2 = std::max(0, std::min(w, x2));
    y1 = std::max(0, std::min(h, y1));
    y2 = std::max(0, std::min(h, y2));
    if (x2 < x1)
        std::swap(x1, x2);
    if (y2 < y1)
        std::swap(y1, y2);

    int hh = y2 - y1,
        ww = x2 - x1;
    CORE_ASSERT_TRUE_S(hh >= 0 && ww >= 0);
    corecvs::Matrix res(hh, ww);
    for (int i = y1; i < y2; ++i)
    {
        for (int j = rowPointers[i]; j < rowPointers[i + 1]; ++j)
        {
            if (columns[j] >= x2)
                break;
            if (columns[j] <  x1)
                continue;
            res.a(i - y1, columns[j] - x1) = values[j];
        }
    }
    return res;
}

SparseMatrix::SparseMatrix(const Matrix &dense, double threshold) : h(dense.h), w(dense.w)
{
    rowPointers.resize(h + 1);
    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            double val = dense.a(i, j);
            if (std::abs(val) > threshold)
            {
                values.push_back(val);
                columns.push_back(j);
                rowPointers[i + 1] = (int)values.size();
            }
        }
    }
}

SparseMatrix::SparseMatrix(int h, int w, const std::vector<double> &values, const std::vector<int> &columns, const std::vector<int> &rowPointers) : h(h), w(w), values(values), columns(columns), rowPointers(rowPointers)
{
    CORE_ASSERT_TRUE_S(h + 1 == (int)rowPointers.size());
    CORE_ASSERT_TRUE_S(values.size() == columns.size());
    CORE_ASSERT_TRUE_S(*rowPointers.rbegin() == (int)values.size());
}

SparseMatrix::SparseMatrix(int h, int w, const std::map<std::pair<int, int>, double> &data) : h(h), w(w)
{
    // Here we will use lexicographical order of std::map<std::pair<X,X>,Y>
    rowPointers.resize(h + 1);

    int rowPrev = 0;
    for (auto& val: data)
    {
        auto idx  = val.first;
        auto value= val.second;

        int r = idx.first, c = idx.second;
        CORE_ASSERT_TRUE_S(r < h && c < w);

        if (r != rowPrev)
        {
            for (int i = rowPrev + 1; i <= r; ++i)
                rowPointers[i] = (int)values.size();
            rowPrev = r;
        }

        values.push_back(value);
        columns.push_back(c);
    }
    for (int i = rowPrev + 1; i <= h; ++i)
        rowPointers[i] = (int)values.size();
}

double SparseMatrix::a(int y, int x) const
{
    for (int i = rowPointers[y]; i < rowPointers[y + 1]; ++i)
        if (columns[i] == x)
            return values[i];
    return 0.0;
}

Matrix SparseMatrix::denseRows(int x1, int y1, int x2, int y2, std::vector<int> &colIdx)
{
    x1 = std::max(0, std::min(w, x1));
    x2 = std::max(0, std::min(w, x2));
    y1 = std::max(0, std::min(h, y1));
    y2 = std::max(0, std::min(h, y2));
    if (x2 < x1)
        std::swap(x1, x2);
    if (y2 < y1)
        std::swap(y1, y2);

    int h = y2 - y1;

    colIdx.clear();
    std::vector<int> rPtr(h);
    for (int i = 0; i < h; ++i)
    {
        rPtr[i] = rowPointers[i + y1];
        while (columns[rPtr[i]] < x2 && rPtr[i] < rowPointers[i + y1])
            rPtr[i]++;
    }
    std::vector<int> rStartPtr = rPtr;

    while (1)
    {
        int nextCol = -1;
        for (int i = 0; i < h; ++i)
        {
            if (rPtr[i] == rowPointers[i + y1 + 1] || columns[rPtr[i]] >= x2)
                continue;
            if (nextCol < 0)
                nextCol = columns[rPtr[i]];
            nextCol = std::min(nextCol, columns[rPtr[i]]);
        }
        if (nextCol == -1)
            break;
        CORE_ASSERT_TRUE_S(nextCol >= x1 && nextCol < x2);
        colIdx.push_back(nextCol);
        for (int i = 0; i < h; ++i)
            if (rPtr[i] != rowPointers[i + y1 + 1] && columns[rPtr[i]] < x2 && columns[rPtr[i]] == nextCol)
                ++rPtr[i];
    }
    int w = colIdx.size();
    rPtr = rStartPtr;

    corecvs::Matrix M(h, w);
    for (int nnzCols = 0; nnzCols < w; ++nnzCols)
    {
        int nextCol = colIdx[nnzCols];
        CORE_ASSERT_TRUE_S(nextCol >= x1 && nextCol < x2);
        for (int i = 0; i < h; ++i)
            if (rPtr[i] != rowPointers[i + y1 + 1] && columns[rPtr[i]] < x2 && columns[rPtr[i]] == nextCol)
            {
                M.a(i, nnzCols) = values[rPtr[i]];
                ++rPtr[i];
            }
    }
    return M;
}

double& SparseMatrix::a(int y, int x)
{
    int i = 0;
    for (i = rowPointers[y]; i < rowPointers[y + 1] && columns[i] < x; ++i);
    if (i < rowPointers[y + 1] && columns[i] == x)
        return values[i];
    columns.insert(columns.begin() + i, x);
    values.insert(values.begin() + i, 0.0);
    for (int j = y + 1; j <= h; ++j)
        ++rowPointers[j];
    return values[i];
}

SparseMatrix::operator Matrix() const
{
    Matrix m(h, w);

    for (int i = 0; i < h; ++i)
    {
        for (int j = rowPointers[i]; j < rowPointers[i + 1]; ++j)
        {
            m.a(i, columns[j]) = values[j];
        }
    }
    return m;
}

SparseMatrix corecvs::operator -(const SparseMatrix &a)
{
    SparseMatrix b = a;
    for (auto &v: b.values)
        v = -v;
    return b;
}

SparseMatrix corecvs::operator *(const double &lhs, const SparseMatrix &rhs)
{
    SparseMatrix res = rhs;
    for (auto &v: res.values)
        v *= lhs;
    return res;
}

SparseMatrix corecvs::operator *(const SparseMatrix &lhs, const double &rhs)
{
    return rhs * lhs;
}

SparseMatrix corecvs::operator /(const SparseMatrix &lhs, const double &rhs)
{
    SparseMatrix res = lhs;
    for (auto &v: res.values)
        v /= rhs;
    return res;
}

SparseMatrix corecvs::operator +(const SparseMatrix &lhs, const SparseMatrix &rhs)
{
    CORE_ASSERT_TRUE_S(lhs.h == rhs.h && lhs.w == rhs.w);
    std::vector<double> values;
    std::vector<int> columns;
    std::vector<int> rowPointers(lhs.h + 1);
    for (int i = 0; i < lhs.h; ++i)
    {
        int lhs_l = lhs.rowPointers[i], lhs_r = lhs.rowPointers[i + 1];
        int rhs_l = rhs.rowPointers[i], rhs_r = rhs.rowPointers[i + 1];

        int lhs_p = lhs_l, rhs_p = rhs_l;
        while (lhs_p < lhs_r && rhs_p < rhs_r)
        {
            int cl = lhs.columns[lhs_p], cr = rhs.columns[rhs_p];
            if (cl < cr)
            {
                values.push_back(lhs.values[lhs_p]);
                columns.push_back(cl);
                ++lhs_p;
                continue;
            }
            if (cr < cl)
            {
                values.push_back(rhs.values[rhs_p]);
                columns.push_back(cr);
                rhs_p++;
                continue;
            }
            values.push_back(lhs.values[lhs_p] + rhs.values[rhs_p]);
            columns.push_back(cl);
            ++rhs_p;
            ++lhs_p;
        }
        while (lhs_p < lhs_r)
        {
            values.push_back(lhs.values[lhs_p]);
            columns.push_back(lhs.columns[lhs_p]);
            ++lhs_p;
        }
        while (rhs_p < rhs_r)
        {
            values.push_back(rhs.values[rhs_p]);
            columns.push_back(rhs.columns[rhs_p]);
            ++rhs_p;
        }
        rowPointers[i + 1] = (int)values.size();
    }
    return SparseMatrix(lhs.h, lhs.w, values, columns, rowPointers);
}

SparseMatrix corecvs::operator -(const SparseMatrix &lhs, const SparseMatrix &rhs)
{
    CORE_ASSERT_TRUE_S(lhs.h == rhs.h && lhs.w == rhs.w);
    std::vector<double> values;
    std::vector<int> columns;
    std::vector<int> rowPointers(lhs.h + 1);
    for (int i = 0; i < lhs.h; ++i)
    {
        int lhs_l = lhs.rowPointers[i], lhs_r = lhs.rowPointers[i + 1];
        int rhs_l = rhs.rowPointers[i], rhs_r = rhs.rowPointers[i + 1];

        int lhs_p = lhs_l, rhs_p = rhs_l;
        while (lhs_p < lhs_r && rhs_p < rhs_r)
        {
            int cl = lhs.columns[lhs_p], cr = rhs.columns[rhs_p];
            if (cl < cr)
            {
                values.push_back(lhs.values[lhs_p]);
                columns.push_back(cl);
                ++lhs_p;
                continue;
            }
            if (cr < cl)
            {
                values.push_back(-rhs.values[rhs_p]);
                columns.push_back(cr);
                rhs_p++;
                continue;
            }
            values.push_back(lhs.values[lhs_p] - rhs.values[rhs_p]);
            columns.push_back(cl);
            ++rhs_p;
            ++lhs_p;
        }
        while (lhs_p < lhs_r)
        {
            values.push_back(lhs.values[lhs_p]);
            columns.push_back(lhs.columns[lhs_p]);
            ++lhs_p;
        }
        while (rhs_p < rhs_r)
        {
            values.push_back(-rhs.values[rhs_p]);
            columns.push_back(rhs.columns[rhs_p]);
            ++rhs_p;
        }
        rowPointers[i + 1] = (int)values.size();
    }
    return SparseMatrix(lhs.h, lhs.w, values, columns, rowPointers);
}

Vector corecvs::operator *(const SparseMatrix &lhs, const Vector &rhs)
{
    CORE_ASSERT_TRUE_S(lhs.w == rhs.size());
    Vector ans(lhs.h);
    for (int i = 0; i < lhs.h; ++i)
    {
        double res = 0.0;
        for (int j = lhs.rowPointers[i]; j < lhs.rowPointers[i + 1]; ++j)
            res += lhs.values[j] * rhs[lhs.columns[j]];
        ans[i] = res;
    }
    return ans;
}

Vector corecvs::operator *(const Vector &lhs, const SparseMatrix &rhs)
{
    CORE_ASSERT_TRUE_S(lhs.size() == rhs.h);
    Vector ans(rhs.w);
    for (int i = 0; i < rhs.w; ++i)
        ans[i] = 0.0;

    for (int i = 0; i < rhs.h; ++i)
    {
        for (int j = rhs.rowPointers[i]; j < rhs.rowPointers[i + 1]; ++j)
            ans[rhs.columns[j]] += rhs.values[j] * lhs[i];
    }
    return ans;
}

#include <fstream>
SparseMatrix corecvs::operator *(const SparseMatrix &lhs, const SparseMatrix &rhst)
{
    CORE_ASSERT_TRUE_S(lhs.w == rhst.h);
#ifndef WITH_MKL
    auto& rhs = rhst;
    int N = std::max(std::max(lhs.h, lhs.w), std::max(rhs.h, rhs.w));
    std::vector<double> values, acc(N);
    std::vector<int> columns, rowPointers(lhs.h + 1), index(N, -10);
    int h = lhs.h;
    int w = rhs.w;

    for (int i = 0; i < h; ++i)
    {
        int ii = -1, l = 0;
        for (int jj = lhs.rowPointers[i]; jj < lhs.rowPointers[i + 1]; ++jj)
        {
            int j = lhs.columns[jj];
            for (int k = rhs.rowPointers[j]; k < rhs.rowPointers[j + 1]; ++k)
            {
                if (index[rhs.columns[k]] == -10)
                {
                    index[rhs.columns[k]] = ii;
                    ii = rhs.columns[k];
                    ++l;
                }
            }
        }
        rowPointers[i + 1] = rowPointers[i] + l;
        columns.resize(rowPointers[i + 1]);
        values.resize(rowPointers[i + 1]);
        for (int j = rowPointers[i]; j < rowPointers[i + 1]; ++j)
        {
            columns[j] = ii;
            ii = index[ii];
            index[columns[j]] = -10;
        }
        index.clear();
        index.resize(N, -10);
    }
    rowPointers[0] = 0;

    for (int i = 0; i < h; ++i)
    {
        std::sort(&columns[rowPointers[i]], &columns[rowPointers[i + 1]]);
        for (int jj = lhs.rowPointers[i]; jj < lhs.rowPointers[i + 1]; ++jj)
        {
            int j = lhs.columns[jj];
            double val = lhs.values[jj];
            for (int k = rhs.rowPointers[j]; k < rhs.rowPointers[j + 1]; ++k)
            {
                acc[rhs.columns[k]] += val * rhs.values[k];
            }
        }
        for (int j = rowPointers[i]; j < rowPointers[i + 1]; ++j)
        {
            values[j] = acc[columns[j]];
            acc[columns[j]] = 0.0;
        }
    }
    return SparseMatrix(h, w, values, columns, rowPointers);
#else
    auto lhs_mkl = (sparse_matrix_t)lhs;
    auto rhs_mkl = (sparse_matrix_t)rhst;
    sparse_matrix_t res;
    mkl_sparse_spmm(SPARSE_OPERATION_NON_TRANSPOSE, lhs_mkl, rhs_mkl, &res);
    mkl_sparse_destroy(lhs_mkl);
    mkl_sparse_destroy(rhs_mkl);
    SparseMatrix ress(res);
    mkl_sparse_destroy(res);
    return ress;
#endif
}

SparseMatrix SparseMatrix::ata() const
{
#if 1
#ifndef WITH_MKL
    return t() * (*this);
#else
    auto lhs_mkl = (sparse_matrix_t)(*this);
    sparse_matrix_t res;
    mkl_sparse_spmm(SPARSE_OPERATION_TRANSPOSE, lhs_mkl, lhs_mkl, &res);
    mkl_sparse_destroy(lhs_mkl);
    SparseMatrix ress(res);
    mkl_sparse_destroy(res);
    return ress;
#endif
#else
#if 0
//    const auto& rhs = *this;
    std::vector<double> values;
    std::vector<int> columns, rowPointers(h + 1);
    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < h; ++j)
        {
            double total = 0.0;
            int cnt = 0;
            int lhs_l = rowPointers[i], lhs_r = rowPointers[i + 1];
            int rhs_l = rowPointers[j], rhs_r = rowPointers[j + 1];
            while (lhs_l < lhs_r && rhs_l < rhs_r)
            {
                int cl = columns[lhs_l], cr = columns[rhs_l];
                if (cl < cr)
                {
                    ++lhs_l;
                    continue;
                }
                if (cr < cl)
                {
                    ++rhs_l;
                    continue;
                }
                total += values[lhs_l] * values[rhs_l];
                lhs_l++;
                rhs_l++;
                cnt++;
            }
            if (cnt > 0 && total != 0.0)
            {
                values.push_back(total);
                columns.push_back(j);
            }
        }
        rowPointers[i + 1] = values.size();
    }
    return SparseMatrix(h, h, values, columns, rowPointers);
#else
    return t() * (*this);
#endif
#endif
}

SparseMatrix SparseMatrix::t() const
{
#if 0
    std::map<std::pair<int, int>, double> map;
    for (int i = 0; i < h; ++i)
    {
        for (int j = rowPointers[i]; j < rowPointers[i + 1]; ++j)
        {
            map[std::make_pair(columns[j], i)] = values[j];
        }
    }
    return SparseMatrix(w, h, map);
#else
    std::vector<int> tRowPointers(w + 1);
    std::vector<double> tValues(values.size());
    std::vector<int> tColumns(values.size());

    for (auto& id: tRowPointers)
        id = 0;
    tRowPointers[0] = 0;
    for (int i = 0; i < h; ++i)
        for (int j = rowPointers[i]; j < rowPointers[i + 1]; ++j)
            tRowPointers[columns[j]+1]++;
    for (int i = 0; i < w; ++i)
        tRowPointers[i + 1] += tRowPointers[i];
    for (int i = 0; i < h; ++i)
        for (int j = rowPointers[i]; j < rowPointers[i + 1]; ++j)
        {
            int idx = columns[j];
            tColumns[tRowPointers[idx]] = i;
            tValues[tRowPointers[idx]] = values[j];
            tRowPointers[idx]++;
        }
    for (int i = w - 1; i >= 1; --i)
        tRowPointers[i] = tRowPointers[i - 1];
    tRowPointers[0] = 0;
    *tRowPointers.rbegin() = (int)tValues.size();

    return SparseMatrix(w, h, tValues, tColumns, tRowPointers);
#endif
}

#ifdef WITH_MKL

SparseMatrix::operator sparse_matrix_t() const
{
    sparse_matrix_t mkl_sparse;
    mkl_sparse_d_create_csr(&mkl_sparse, SPARSE_INDEX_BASE_ZERO, h, w, const_cast<int*>(&rowPointers[0]), const_cast<int*>(&rowPointers[1]), const_cast<int*>(&columns[0]), const_cast<double*>(&values[0]));
    return mkl_sparse;
}

SparseMatrix::SparseMatrix(const sparse_matrix_t &mklSparse)
{
    double *vals;
    int *rowB, *rowE, *cols;
    int mh, mw;
    sparse_index_base_t indexType;
    sparse_matrix_t copy;

    mkl_sparse_convert_csr(mklSparse, SPARSE_OPERATION_NON_TRANSPOSE, &copy);
    mkl_sparse_d_export_csr(copy, &indexType, &mh, &mw, &rowB, &rowE, &cols, &vals);

    CORE_ASSERT_TRUE_S(indexType == SPARSE_INDEX_BASE_ZERO);

    rowPointers.resize(mh + 1);
    h = mh;
    w = mw;

    for (int i = 0; i < mh; ++i)
    {
        for (int j = rowB[i]; j < rowE[i]; ++j)
        {
            values.push_back(vals[j]);
            columns.push_back(cols[j]);
        }
        rowPointers[i + 1] = (int)values.size();
    }

    mkl_sparse_destroy(copy);
}

#endif // WITH_MKL

SparseMatrix SparseMatrix::upper() const
{
    SparseMatrix copy(*this);
    int valueIdx = 0;
    for (int i = 0; i < h; ++i)
    {
        bool diagExists = false;
        for (int j = rowPointers[i]; j < rowPointers[i + 1]; ++j)
        {
            if (columns[j] >= i)
            {
                if (columns[j] == i)
                    diagExists = true;
                if (columns[j] > i && !diagExists)
                {
                    copy.values.resize(copy.values.size() + 1);
                    copy.columns.resize(copy.columns.size() + 1);
                    copy.values[valueIdx] = 0.0;
                    copy.columns[valueIdx] = i;
                    diagExists = true;
                    valueIdx++;
                }
                copy.values[valueIdx] = values[j];
                copy.columns[valueIdx] = columns[j];
                valueIdx++;
            }
        }
        copy.rowPointers[i + 1] = valueIdx;
    }
    copy.values.resize(valueIdx);
    copy.columns.resize(valueIdx);
    return copy;
}

bool SparseMatrix::linSolve(const Vector &rhs, Vector &res, bool symmetric, bool posDef) const
{
    if (symmetric)
#ifdef WITH_MKL
        return LinSolve(symmetric ? upper() : (*this), rhs, res, true, posDef);
#else
        return LinSolve(*this, rhs, res, true, posDef);
#endif
    return LinSolve(*this, rhs, res, false, posDef);
}

bool SparseMatrix::LinSolve(const SparseMatrix &m, const Vector &rhs, Vector &res, bool symmetric, bool posDef)
{
    CORE_ASSERT_TRUE_S(rhs.size() == m.h);
#ifndef WITH_MKL
    CORE_UNUSED(posDef);
    return Matrix::LinSolve((Matrix)m, rhs, res, symmetric);
#else
    res = Vector(m.w);
    auto& sol = res;
    for (int i = 0; i < m.w; ++i)
        sol[i] = 0.0;
    _MKL_DSS_HANDLE_t dss_handle;
    int options = MKL_DSS_DEFAULTS + MKL_DSS_ZERO_BASED_INDEXING;
    int nnz     = (int)m.values.size();
    int retval  = dss_create(dss_handle, options);
    int order   = MKL_DSS_AUTO_ORDER;
    int *pivot  = 0;
    int nrhs    = 1;
    int solve_options = MKL_DSS_DEFAULTS;
    int delOptions = 0;

    CORE_ASSERT_TRUE_S(retval == MKL_DSS_SUCCESS);
    options = symmetric ? MKL_DSS_SYMMETRIC : MKL_DSS_NON_SYMMETRIC;

    retval = dss_define_structure(dss_handle, options, &m.rowPointers[0], m.h, m.w, &m.columns[0], nnz);
    CORE_ASSERT_TRUE_S(retval == MKL_DSS_SUCCESS);

    retval = dss_reorder(dss_handle, order, pivot);
    CORE_ASSERT_TRUE_S(retval == MKL_DSS_SUCCESS);

    int decompose_options = symmetric ?  posDef ? MKL_DSS_POSITIVE_DEFINITE : MKL_DSS_INDEFINITE : 0;
    retval = dss_factor_real(dss_handle, decompose_options, &m.values[0]);
    if (retval != MKL_DSS_SUCCESS)
    {
       dss_delete(dss_handle, delOptions);
       return false;
    }


    retval = dss_solve_real(dss_handle, solve_options, &rhs[0], nrhs, &sol[0]);
    if (retval != MKL_DSS_SUCCESS)
    {
       dss_delete(dss_handle, delOptions);
       return false;
    }

    dss_delete(dss_handle, delOptions);
    return true;
#endif // WITH_MKL
}

std::ostream& corecvs::operator<< (std::ostream &out, const SparseMatrix &sm)
{
    out << (Matrix)sm;
    return out;
}

void SparseMatrix::print(std::ostream& out) const
{
    out << *this;
}

bool corecvs::SparseMatrix::LinSolveSchurComplement(const corecvs::SparseMatrix &M, const corecvs::Vector &Bv, const std::vector<int> &diagBlocks, corecvs::Vector &res, bool symmetric, bool posDef)
{
    /*
     * So we partition M and B into
     * +---+---+   /   \   /   \
     * | A | B |   | x |   | a |
     * +---+---+ * +---+ = +---+
     * | C | D |   | y |   | b |
     * +---+---+   \   /   \   /
     * Where D is block-diagonal well-conditioned matrix
     *
     * Then we invert D explicitly and solve
     * x = (A-BD^{-1}C)^{-1}(a-BD^{-1}b)
     * y = D^{-1}(b-Cx)
     *
     * Note that M is symmetric => D is symmetric, (A-BD^{-1}C) is symmetric
     *           M is posdef    => D is posdef,    (A-BD^{-1}C) is symmetric (TODO: isposdef)
     */

    auto Ah = diagBlocks[0],
         Aw = diagBlocks[0];
    auto Bw = M.w - Aw,
         Bh = Ah;
    auto Cw = Aw,
         Ch = M.h - Ah;
    auto Dw = Bw;

#ifndef WITH_BLAS
#error NIY
#else
    /*
     * The same as above, but with fancy LAPACK
     */
    auto N = diagBlocks.size() - 1;
    std::vector<int> pivots(Dw), pivotIdx(N);

    std::vector<corecvs::Matrix> qrd(N);
    auto startBlocks = std::chrono::high_resolution_clock::now();
    corecvs::parallelable_for(0, (int)N, [&](const corecvs::BlockedRange<int> &r)
            {
                for (int i = r.begin(); i != r.end(); ++i)
                {
                    qrd[i] = M.denseSubMatrix(diagBlocks[i], diagBlocks[i], diagBlocks[i + 1], diagBlocks[i + 1]);
                    pivotIdx[i] = diagBlocks[i] - diagBlocks[0];
                }
            });
    auto  stopBlocks = std::chrono::high_resolution_clock::now();

    auto  startDiagFactor = std::chrono::high_resolution_clock::now();
    // Factorizing (without "\"")
    corecvs::parallelable_for(0, (int)N, [&](const corecvs::BlockedRange<int> &r)
            {
                for (int i = r.begin(); i < r.end(); ++i)
                {
                    auto& MM = qrd[i];
                    if (!symmetric)
                    {
                        LAPACKE_dgetrf(LAPACK_ROW_MAJOR, MM.h, MM.w, MM.data, MM.stride, &pivots[pivotIdx[i]]);
                    }
                    else
                    {
                        if (posDef)
                            LAPACKE_dpotrf(LAPACK_ROW_MAJOR, 'U', MM.h, MM.data, MM.stride);
                        else
                            LAPACKE_dsytrf(LAPACK_ROW_MAJOR, 'U', MM.h, MM.data, MM.stride, &pivots[pivotIdx[i]]);
                    }
                }
            });
    auto  stopDiagFactor = std::chrono::high_resolution_clock::now();

    // Computing BD^{-1}
    auto startSparsePrep = std::chrono::high_resolution_clock::now();
    corecvs::SparseMatrix A(M ,   0,  0,      Aw,      Ah),
                          B(M,   Aw,  0, Aw + Bw,      Bh),
                          C(M,    0, Ah,      Cw, Ah + Ch);
    CORE_ASSERT_TRUE_S(A.h == Ah && A.w == Aw);
    CORE_ASSERT_TRUE_S(B.h == Bh && B.w == Bw);
    CORE_ASSERT_TRUE_S(C.h == Ch && C.w == Cw);
    auto stopSparsePrep = std::chrono::high_resolution_clock::now();

    // Computing BD^{-1}
    //recvs::Matrix BDinv(Bh, Dw);
    auto startDinvBt = std::chrono::high_resolution_clock::now();
    corecvs::Matrix DinvtBt = (corecvs::Matrix)B.t();
    CORE_ASSERT_TRUE_S(DinvtBt.h == Dw && DinvtBt.w == Bh);
    corecvs::parallelable_for(0, (int)qrd.size(), [&](const corecvs::BlockedRange<int> &r)
    {
        for (int i = r.begin(); i < r.end(); ++i)
        {
            auto begin = diagBlocks[i], end = diagBlocks[i + 1];
            auto len = end - begin;
            auto& MM = qrd[i];
            auto ptr = &DinvtBt.a(diagBlocks[i] - diagBlocks[0], 0);
            if (!symmetric)
            {
                LAPACKE_dgetrs(LAPACK_ROW_MAJOR, 'T', len, Bh, MM.data, MM.stride, &pivots[pivotIdx[i]], ptr, DinvtBt.stride);
            }
            else
            {
                if (posDef)
                    LAPACKE_dpotrs(LAPACK_ROW_MAJOR, 'U', len, Bh, MM.data, MM.stride, ptr, DinvtBt.stride);
                else
                    LAPACKE_dsytrs(LAPACK_ROW_MAJOR, 'U', len, Bh, MM.data, MM.stride, &pivots[pivotIdx[i]], ptr, DinvtBt.stride);
            }
        }
    });
    auto stopDinvBt = std::chrono::high_resolution_clock::now();
    // Computing lhs/rhs
    auto startLhsRhs = std::chrono::high_resolution_clock::now();
    corecvs::Vector a(Ah, &Bv[0]), b(Ch, &Bv[Ah]), rhs;
    corecvs::Matrix lhs;
    rhs = a - b * DinvtBt;

    lhs = (corecvs::Matrix)A - (corecvs::Matrix)(C.t() * DinvtBt).t();
    auto stopLhsRhs = std::chrono::high_resolution_clock::now();

    auto startX = std::chrono::high_resolution_clock::now();
    // Solving for x
    corecvs::Vector x(Aw), y(Bw);
    bool foo = lhs.linSolve(rhs, x, symmetric, false);
    auto stopX = std::chrono::high_resolution_clock::now();

    if (!foo) return false;

    auto startY = std::chrono::high_resolution_clock::now();
    // Solving for y
    rhs = b - C * x;
    corecvs::parallelable_for(0, (int)qrd.size(), [&](const corecvs::BlockedRange<int> &r)
    {
        for (int i = r.begin(); i < r.end(); ++i)
        {
            auto begin = diagBlocks[i], end = diagBlocks[i + 1];
            auto len = end - begin;
            auto& MM = qrd[i];
            corecvs::Vector bcx(len);
            for (int j = 0; j < len; ++j)
                bcx[j] = rhs[j + begin - Aw];
            if (!symmetric)
            {
                LAPACKE_dgetrs(LAPACK_ROW_MAJOR, 'N', len, 1, MM.data, MM.stride, &pivots[pivotIdx[i]], &bcx[0], 1);
            }
            else
            {
                if (posDef)
                    LAPACKE_dpotrs(LAPACK_ROW_MAJOR, 'U', len, 1, MM.data, MM.stride, &bcx[0], 1);
                else
                    LAPACKE_dsytrs(LAPACK_ROW_MAJOR, 'U', len, 1, MM.data, MM.stride, &pivots[pivotIdx[i]], &bcx[0], 1);
            }
            auto res = bcx;
            for (int j = begin; j < end; ++j)
                y[j - Aw] = res[j - begin];
        }
    });
    auto stopY = std::chrono::high_resolution_clock::now();

    double blocks = (stopBlocks - startBlocks).count(),
           diagFac= (stopDiagFactor - startDiagFactor).count(),
           sparseP= (stopSparsePrep - startSparsePrep).count(),
           dinvBt = (stopDinvBt - startDinvBt).count(),
           lhsRhs = (stopLhsRhs - startLhsRhs).count(),
           xxx    = (stopX - startX).count(),
           yyy    = (stopY - startY).count();
    static int statcnt = 0;
    statcnt++;
    if (statcnt % 1000 == 0)
    {
    double total = blocks + diagFac + sparseP + dinvBt + lhsRhs + xxx + yyy;
    total /= 100.0;
#define SPRINT(T, V) \
        << "\t" << T << ": " << V / total << " % / " << V << "ns" << std::endl
    std::cout << "LSC Stats: " << total * 100.0 << "ns"  << std::endl
        SPRINT("Blocks", blocks)
        SPRINT("DiagFactor", diagFac)
        SPRINT("SparsePrep", sparseP)
        SPRINT("DinvBt", dinvBt)
        SPRINT("LhsRhs", lhsRhs)
        SPRINT("X", xxx)
        SPRINT("Y", yyy);
    }

    for (int i = 0; i < Aw; ++i)
        res[i] = x[i];
    for (int j = 0; j < Bw; ++j)
        res[j + Aw] = y[j];
    return true;
#endif
}

bool        corecvs::SparseMatrix::linSolveSchurComplement(const corecvs::Vector &B, const std::vector<int> &diagBlocks, corecvs::Vector &res, bool symmetric, bool posDef)
{
    return corecvs::SparseMatrix::LinSolveSchurComplement(*this, B, diagBlocks, res, symmetric, posDef);
}
