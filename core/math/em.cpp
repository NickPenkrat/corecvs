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
        parallelable_for(0, N, [&](const corecvs::BlockedRange<int> &r){
        for (int i = r.begin(); i != r.end(); ++i)
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
                std::cout << "^" << std::flush;
                std::random_device rd;
                int id = std::uniform_int_distribution<int>(0, M - 1)(rd);
                for (int j = 0; j < K; ++j)
                    m[j] = A.a(id, j);
                cnt = 1;
            }
            m = m / cnt;
        }});

        std::atomic<double> sdist(0.0);
        parallelable_for(0, M, [&](const corecvs::BlockedRange<int> &r){
        for (int i = r.begin(); i != r.end(); ++i)
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
            for (int j = 0; j < N; ++j)
                probabilities.a(i, j) = (j == maxId) ? 1.0 : 0.0;
            double val, inc;
            do
            {
               val = sdist;
               inc = val + minDist;

            } while (!sdist.compare_exchange_strong(val, inc));
        }});
        std::cout << sdist << std::flush;
        if (prev == sdist)
            break;
        prev = sdist;
    }
    std::cout << std::endl;
    stepM();
}

EM::EM(const Matrix &A, int N, bool smooth, int maxIter, bool randomInit) :
    N(N), M(A.h), K(A.w), smooth(smooth), A(A), maxIter(maxIter)
{
    CORE_ASSERT_TRUE_S(M > N);

    std::mt19937 rng((std::random_device())());
    std::uniform_int_distribution<int> rint(0, N - 1);
    std::uniform_real_distribution<double> rdouble(0, 1);

    probabilities = Matrix(M, N);
    prior.resize(N);
    means.resize(N);
    covariances.resize(N);
    for (int i = 0; i < N; ++i)
    {
        prior[i] = 1.0 / N;
        means[i] = Vector(K);
        covariances[i] = Matrix(K, K);
    }


    if (randomInit)
    {
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

        for (int i = 0; i < M; ++i)
            probabilities.a(i, rint(rng)) = 1.0;
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
               probabilities.a(indicies[i], j) = 0.0;
            probabilities.a(indicies[i], i)  = 1.0;
        }
    }
    else
    {
        std::vector<double> distances(M);
        auto idrng = std::uniform_int_distribution<int>(0, M - 1);
        int id1 = idrng(rng);
        for (int j = 0; j < K; ++j)
            means[0][j] = A.a(id1, j);
        for (int cl = 1; cl < N; ++cl)
        {
            double sdist = 0.0;
            for (int i = 0; i < M; ++i)
            {
                double dist = 0.0;
                for (int j = 0; j < K; ++j)
                    dist += (means[cl - 1][j] - A.a(i, j)) * (means[cl - 1][j] - A.a(i, j));
                distances[i] = std::min(distances[i], dist);
                sdist += distances[i];
            }
            double alpha = sdist * rdouble(rng);
            double s = distances[0];
            for (int i = 0; i < M; ++i)
                if (s >= alpha)
                {
                    for (int j = 0; j < K; ++j)
                        means[cl][j] = A.a(i, j);
                    break;
                }
                else
                    s += distances[i + 1];
        }
        std::atomic<double> sdist(0.0);
        parallelable_for(0, M, [&](const corecvs::BlockedRange<int> &r){
        for (int i = r.begin(); i != r.end(); ++i)
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
            for (int j = 0; j < N; ++j)
                probabilities.a(i, j) = (j == maxId) ? 1.0 : 0.0;
            double val, inc;
            do
            {
               val = sdist;
               inc = val + minDist;

            } while (!sdist.compare_exchange_strong(val, inc));
        }});
    }
    runKMeans();
    runEM();
}

void EM::runEM(int mIter)
{
    if (mIter < 0)
        mIter = maxIter;
    auto prev = std::numeric_limits<double>::max();
    for (int i = 0; i < mIter; ++i)
    {
        std::cout << "$" << std::flush;
        stepM();
        auto res = stepE();
        if (res >= prev)
            break;
        prev = res;
    }
    std::cout << std::endl;
}

double EM::stepE()
{
    std::atomic<double> log(0.0);
    parallelable_for(0, N, [&](const BlockedRange<int> &r){
    for (int i = r.begin(); i != r.end(); ++i)
    {
        auto& mean = means[i];
        auto& cov  = covariances[i];
        auto& p    = prior[i];

        Matrix U(cov), V(K, K), D(1, K);
        Matrix::svd(&U, &D, &V);

        double det = 1.0, ldet = 0.0;
        for (int ii = 0; ii < K; ++ii)
        {
            det *= D.a(0, ii);
            ldet += std::log(D.a(0, ii));
        }

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

            probabilities.a(j, i) = p / std::sqrt(det) * std::exp( -w / 2.0);
            double val, pval, ilog = std::log(p) - 0.5 * ldet - w / 2.0;
            do
            {
                val = log;
                pval = log + ilog;
            } while(!log.compare_exchange_strong(val, pval));
        }
    }});
    parallelable_for(0, M, [&](const BlockedRange<int> &r){
    for (int i = r.begin(); i != r.end(); ++i)
    {
        double sum = 0.0;
        for (int j = 0; j < N; ++j)
            sum += probabilities.a(i, j);
        if (sum == 0.0)
        {
            std::cout << "." << std::flush;
            std::random_device rd;
            int id = std::uniform_int_distribution<int>(0, N - 1)(rd);
            probabilities.a(i, id) = 1;
            sum = 1.0;
        }
        CORE_ASSERT_TRUE_S(!std::isnan(sum));
        for (int j = 0; j < N; ++j)
            probabilities.a(i, j) = probabilities.a(i, j) / sum;
    }});
    return log;
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
//    CORE_ASSERT_TRUE_S(std::abs(ps - M) / M < 1e-9);
    for (int i = 0; i < N; ++i)
        prior[i] /= M;
}

}
