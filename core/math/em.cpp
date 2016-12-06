#include <random>
#include <cmath>
#include <set>

#include "em.h"

namespace corecvs
{

std::ostream& operator<<(std::ostream& os, const EM &em)
{
    os << "GM with " << em.N << " components" << std::endl;
    os << "Probabilities: " << std::endl;
    os << "\t";
    for (auto& p: em.prior)
        os << p << " ";
    os << std::endl << "Means: " << std::endl;
    for (auto& v: em.means)
        os << "\t" << v << std::endl;
    os << "Covariances: " << std::endl;
    for (auto& c: em.covariances)
        os << c << std::endl;
    os << std::endl;
    return os;
}

void EM::runKMeans()
{
    double prev = std::numeric_limits<double>::max();
    for (int it = 0; it < maxIter; ++it)
    {
        std::cout << "|" << std::flush;
        for (int i = 0; i < N; ++i)
        {
            auto& m = means[i];
            for (int j = 0; j < K; ++j)
                m[j] = 0.0;
            int cnt = 0;
            for (int j = 0; j < M; ++j)
                if (probabilities.a(j, i) != 0.0)
                {
                    for (int k = 0; k < K; ++k)
                        m[k] += A.a(j, k);
                    cnt++;
                }
            if (cnt == 0)
            {
                std::cout << "CLUSTER DEGENERATED" << std::endl;
                std::random_device rd;
                int id = std::uniform_int_distribution<int>(0, M - 1)(rd);
                for (int j = 0; j < K; ++j)
                    m[j] = A.a(id, j);
                cnt = 1;
            }
            m = m / cnt;
        }

        double sdist = 0.0;
        for (int i = 0; i < M; ++i)
        {
            Vector vv(K);
            for (int j = 0; j < K; ++j)
                vv[j] = A.a(i, j);

            int maxId = 0;
            double minDist = (means[0] - vv).sumAllElementsSq();
            for (int j = 1; j < N; ++j)
                if ((means[j] - vv).sumAllElementsSq() < minDist)
                {
                    minDist = (means[j] - vv).sumAllElementsSq();
                    maxId = j;
                }
            sdist += minDist;
            for (int j = 0; j < N; ++j)
                probabilities.a(i, j) = (j == maxId) ? 1.0 : 0.0;
        }
        if (prev == sdist)
            break;
        prev = sdist;
    }
    std::cout << std::endl;
    stepM();
}

EM::EM(const Matrix &A, int N, bool smooth, int maxIter) :
    N(N), M(A.h), K(A.w), smooth(smooth), A(A), maxIter(maxIter)
{
    CORE_ASSERT_TRUE_S(M > N);

    std::mt19937 rng((std::random_device())());
    std::uniform_int_distribution<int> rint(0, N - 1);
    std::uniform_real_distribution<double> rdouble(0, 1);

    std::vector<int> indicies;
    indicies.resize(N);
    for (int i = 0; i < N; ++i)
        indicies[i] = i;
    for (int i = N; i < M; ++i)
    {
        auto alpha = rdouble(rng);
        alpha *= (i + 1.0);
        if (alpha < N)
            indicies[alpha] = i;
    }
    std::set<int> set(indicies.begin(), indicies.end());
    std::shuffle(indicies.begin(), indicies.end(), rng);

    probabilities = Matrix(M, N);

    for (int i = 0; i < M; ++i)
        probabilities.a(i, rint(rng)) = 1.0;

    prior.resize(N);
    means.resize(N);
    covariances.resize(N);
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
            probabilities.a(indicies[i], j) = 0.0;
        probabilities.a(indicies[i], i)  = 1.0;
        prior[i] = 1.0 / N;
        means[i] = Vector(K);
        covariances[i] = Matrix(K, K);
    }

    runKMeans();
    runEM();
}

void EM::runEM(int mIter)
{
    if (mIter < 0)
        mIter = maxIter;
    for (int i = 0; i < mIter; ++i)
    {
        std::cout << "$" << std::flush;
        stepM();
        stepE();
    }
    std::cout << std::endl;
}

void EM::stepE()
{
    double log = 0.0;
    for (int i = 0; i < N; ++i)
    {
        auto& mean = means[i];
        auto& cov  = covariances[i];
        auto& p    = prior[i];

        Matrix U(cov), V(K, K), D(1, K);
        Matrix::svd(&U, &D, &V);

        double det = 1.0;
        for (int ii = 0; ii < K; ++ii)
            det /= D.a(0, ii);

        for (int j = 0; j < M; ++j)
        {
            Vector vv(K);
            for (int ii = 0; ii < K; ++ii)
                vv[ii] = A.a(j, ii);
            vv -= mean;

            auto utv = vv * U;
            double w = 0.0;

            for (int iii = 0; iii < K; ++iii)
            {
                auto dd = utv[iii] * utv[iii] / D.a(0, iii);
                w += dd;
            }

            probabilities.a(j, i) = p * std::sqrt(det) * std::exp( -w / 2.0);
        }
    }
    for (int i = 0; i < M; ++i)
    {
        double sum = 0.0;
        for (int j = 0; j < N; ++j)
            sum += probabilities.a(i, j);
        if (sum == 0.0)
        {
            std::cout << "POINT DIVERGED" << std::endl;
            std::random_device rd;
            int id = std::uniform_int_distribution<int>(0, M - 1)(rd);
            probabilities.a(i, id) = 1;
            sum = 1.0;
        }
        CORE_ASSERT_TRUE_S(!std::isnan(sum));
        for (int j = 0; j < N; ++j)
            probabilities.a(i, j) = probabilities.a(i, j) / sum;
    }
}

void EM::stepM()
{
    double ps = 0.0;
    if (!smooth)
    {
        for (int i = 0; i < M; ++i)
        {
            double max = probabilities.a(i, 0);
            int maxId = 0;
            for (int j = 0; j < N; ++j)
                if (probabilities.a(i, j) > max)
                {
                    max = probabilities.a(i, j);
                    maxId = j;
                }
            for (int j = 0; j < N; ++j)
                probabilities.a(i, j) = maxId == j ? 1.0 : 0.0;
        }
    }
    for (int i = 0; i < N; ++i)
    {
        auto& mean = means[i];
        auto& cov  = covariances[i];
        auto& p    = prior[i];

        p = 0.0;
        for (int j = 0; j < K; ++j)
        {
            for (int k = 0; k < K; ++k)
                cov.a(j, k) = 0.0;
            mean[j] = 0;
        }

        for (int k = 0; k < M; ++k)
        {
            p += probabilities.a(k, i);
            CORE_ASSERT_TRUE_S(!std::isnan(probabilities.a(k, i)));
            CORE_ASSERT_TRUE_S(!std::isnan(p));
            for (int j = 0; j < K; ++j)
                mean[j] += probabilities.a(k, i) * A.a(k, j);
        }
        if (p <= 0.0)
        {
            std::cout << "DIVERGED!" << std::endl;
            p = 1 / M;
            std::random_device rd;
            int id = std::uniform_int_distribution<int>(0, M - 1)(rd);
            for (int j = 0; j < K; ++j)
            {
                mean[j] = A.a(id, j);
                cov.a(j, j) = 1.0;
            }
            continue;
        }
        mean /= p;

        for (int k = 0; k < M; ++k)
        {
            auto pp = probabilities.a(k, i) / p;
            for (int j = 0; j < K; ++j)
                for (int l = 0; l < K; ++l)
                    cov.a(j, l) += pp * (A.a(k, j) - mean[j]) * (A.a(k, l) - mean[l]);
        }
        ps += p;
    }
    CORE_ASSERT_TRUE_S(std::abs(ps - M) / M < 1e-9);
    for (int i = 0; i < N; ++i)
        prior[i] /= M;
}

}
