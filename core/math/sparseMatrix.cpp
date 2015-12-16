#include "sparseMatrix.h"

using namespace corecvs;

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
                rowPointers[i + 1] = values.size();
            }
        }
    }
}

SparseMatrix::SparseMatrix(int h, int w, const std::vector<double> &values, const std::vector<int> &columns, const std::vector<int> &rowPointers) : h(h), w(w), values(values), columns(columns), rowPointers(rowPointers)
{
    CORE_ASSERT_TRUE_S(h + 1 == rowPointers.size());
    CORE_ASSERT_TRUE_S(values.size() == columns.size());
    CORE_ASSERT_TRUE_S(*rowPointers.rbegin() == values.size());
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
                rowPointers[i] = values.size();
            rowPrev = r;
        }

        values.push_back(value);
        columns.push_back(c);
    }
    for (int i = rowPrev + 1; i <= h; ++i)
        rowPointers[i] = values.size();
}

double SparseMatrix::a(int y, int x) const
{
    for (int i = rowPointers[y]; i < rowPointers[y + 1]; ++i)
        if (columns[i] == x)
            return values[i];
    return 0.0;
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
        rowPointers[i + 1] = values.size();
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
        rowPointers[i + 1] = values.size();
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

SparseMatrix corecvs::operator *(const SparseMatrix &lhs, const SparseMatrix &rhst)
{
    CORE_ASSERT_TRUE_S(lhs.w == rhst.h);
    auto rhs = rhst.t();
    std::vector<double> values;
    std::vector<int> columns, rowPointers(lhs.h + 1);
    for (int i = 0; i < lhs.h; ++i)
    {
        for (int j = 0; j < rhst.w; ++j)
        {
            double total = 0.0;
            int cnt = 0;
            int lhs_l = lhs.rowPointers[i], lhs_r = lhs.rowPointers[i + 1];
            int rhs_l = rhs.rowPointers[j], rhs_r = rhs.rowPointers[j + 1];
            while (lhs_l < lhs_r && rhs_l < rhs_r)
            {
                int cl = lhs.columns[lhs_l], cr = rhs.columns[rhs_l];
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
                total += lhs.values[lhs_l] * rhs.values[rhs_l];
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
    return SparseMatrix(lhs.h, rhs.w, values, columns, rowPointers);
}

SparseMatrix SparseMatrix::t() const
{
    std::map<std::pair<int, int>, double> map;
    for (int i = 0; i < h; ++i)
    {
        for (int j = rowPointers[i]; j < rowPointers[i + 1]; ++j)
        {
            map[std::make_pair(columns[j], i)] = values[j];
        }
    }
    return SparseMatrix(w, h, map);
}

#ifdef WITH_MKL
SparseMatrix::operator sparse_matrix_t()
{
    sparse_matrix_t mkl_sparse;
    mkl_sparse_d_create_csr(&mkl_sparse, SPARSE_INDEX_BASE_ZERO, h, w, &rowPointers[0], &rowPointers[1], &columns[0], &values[0]);
    return mkl_sparse;
}
#endif

std::ostream& corecvs::operator<< (std::ostream &out, const SparseMatrix &sm)
{
    out << (Matrix)sm;
    return out;
}

void SparseMatrix::print(std::ostream& out) const
{
    out << *this;
}
