#include <cmath>
#include <chrono>
#include <iomanip>
#include <map>

#include "vector.h"
#include "cblasLapackeWrapper.h"

namespace corecvs
{
enum class MinresQLPStatus
{
    RUNNING    = -2,
    BOTH_EIGEN = -1,
    ZERO       =  0,
    SOLVED_RTOL=  1,
    MINLEN_RTOL=  2,
    SOLVED_EPS =  3,
    MINLEN_EPS =  4,
    EIGEN      =  5,
    XNORM      =  6,
    ACOND      =  7,
    ITERATION  =  8,
    LSQ_NOCONV =  9
};
std::ostream& operator<<(std::ostream& o, const MinresQLPStatus &s);
struct MinresQLPParams
{
    double eps = 2.3e-16,
           rtol= 1e-16,
           maxXNorm = 1e100,
           ACondLim = 1e100,
           TranCond = 1e7;
    int maxIt = -1;
};

template<typename M>
class MinresQLP : MinresQLPParams
{
    struct AutoTimer
    {
        AutoTimer(MinresQLP *solver, const std::string &tag) : solver(solver), tag(tag), start(std::chrono::high_resolution_clock::now())
        {
        }
        AutoTimer(MinresQLP *solver, std::string&& tag) : solver(solver), tag(tag), start(std::chrono::high_resolution_clock::now())
        {
        }
        ~AutoTimer()
        {
            auto stop = std::chrono::high_resolution_clock::now();
            solver->timings[tag] += (stop - start).count() / 1e9;
        }
        MinresQLP *solver;
        std::string tag;
        decltype(std::chrono::high_resolution_clock::now()) start;
    };
#define AT(t) \
    AutoTimer timer(this, t);
public:
    std::map<std::string, double> timings;
    static MinresQLPStatus Solve(const M &a, const Vector &b, Vector &x, const MinresQLPParams &params = MinresQLPParams())
    {
        MinresQLP solver(a, b, x, params);
        return solver.run();
    }
    MinresQLP (const M &A, const Vector &b, Vector &x, const MinresQLPParams &params = MinresQLPParams()) : MinresQLP(A, b, x, (int)b.size(), params)
    {
    }
    ~MinresQLP()
    {
        double total = 0.0;
        int column = 0;
        for (auto& kv: timings)
        {
            column = std::max(column, (int)kv.first.size());
            total += kv.second;
        }
        for (auto& kv: timings)
        {
            std::cout << std::setw(column + 1) << kv.first << "\t" << kv.second / total * 100.0 << "\t" << kv.second << std::endl;
        }
    }
    MinresQLPStatus run()
    {
        r3 = r2 = b;
        beta1 = !r2;

        flag = MinresQLPStatus::RUNNING;
        rnorm = betan = phi = beta1;

        relres = rnorm / (beta1 + std::numeric_limits<double>::min());
        x = Vector(N);

        if (beta1 == 0.0)
            return;

        auto normBefore = !(A * x - b);
        while (flag == MinresQLPStatus::RUNNING && iter < maxIt)
        {
            if (!singleStep())
                break;
        }
        auto normAfter = !(A * x - b);
        auto relBefore = normBefore / !b, relAfter = normAfter / !b;
        std::cout << normBefore << " (" << relBefore << ") > " << normAfter << " (" << relAfter << ") [" << normBefore / normAfter << "] @ " << iter << std::endl;
        std::cout << "MINRES-QLP status: " << flag << std::endl;

		return flag;
    }
    bool singleStep()
    {
        ++iter;

        if (!residualUpdate())
            return false;

        // Applying rotations
        applyLeftPrev();
        applyLeftCurr();
        applyRightPrev();
        applyRightCurr();

        // Here we go with update
        updateXNorm();

        if (Acond < TranCond && flag == MinresQLPStatus::RUNNING && QLPiter == 0)
            minresStep();
        else
            minresQLPStep();

        computeNextRight();

        estimateNorms();

        checkExit();
        return true;
    }
private:
    MinresQLP (const M &A, const Vector &b, Vector &x, int N, const MinresQLPParams &params) :
        MinresQLPParams(params),
        A(A), b(b), x(x),
        N(N),
        w(N), wl(N), wl2(N), xl2(N),
        r2(b), r3(b), r1(N)
    {
        if (maxIt < 0)
            maxIt = N;
    }

    bool residualUpdate()
    {
        auto betal = beta;
        {
        AT("Residual: v")
        beta  = betan;

#ifndef WITH_BLAS
        v = std::move(r3) / beta;
#else
        v = std::move(r3);
        cblas_dscal(v.size(), 1.0 / beta, &v[0], 1);
#endif
        }
        {
        AT("Residual: spmv")
        r3 = A * v;
        }
        {
        AT("Residual: rem")

        if (iter > 1)
#ifndef WITH_BLAS
            r3 -= (beta / betal) * r1;
#else
            cblas_daxpy(r3.size(), -beta / betal, &r1[0], 1, &r3[0], 1);
#endif

#ifndef WITH_BLAS
        alpha = r3 & v;
#else
        alpha = cblas_ddot(r3.size(), &r3[0], 1, &v[0], 1);
#endif
#ifndef WITH_BLAS
        r3 -= (alpha / beta) * r2;
#else
        cblas_daxpy(r3.size(), -alpha/beta, &r2[0], 1, &r3[0], 1);
#endif
        r1 = std::move(r2);
        r2 = r3;

#ifndef WITH_BLAS
        betan = !r3;
#else
        betan = cblas_dnrm2(r3.size(), &r3[0], 1);
#endif
        if (iter == 1 && betan == 0.0)
        {
            if (alpha == 0.0)
            {
                flag = MinresQLPStatus::ZERO;
                return false;
            }
            flag = MinresQLPStatus::BOTH_EIGEN;
#ifndef WITH_BLAS
            x = b / alpha;
#else
            x = b;
            cblas_dscal(x.size(), 1.0 / alpha, &x[0], 1);

#endif
            return false;
        }

        pnorm = std::sqrt(betal * betal + alpha * alpha + betan * betan);
        }
        return true;
    }
    void applyLeftPrev()
    {
        AT("Left prev")
        auto dbar = dltan;
        dlta = cs * dbar + sn * alpha;
        epln = eplnn;
        gbar = sn * dbar - cs * alpha;
        eplnn = sn * betan;
        dltan = -cs * betan;
        dlta_QLP = dlta;
    }
    void applyLeftCurr()
    {
        AT("Left curr")
        gammal3 = gammal2;
        gammal2 = gammal;
        gammal = gamma;
        givens2(gbar, betan, cs, sn, gamma);
        gamma_tmp = gamma;
        taul2 = taul;
        taul = tau;
        tau = cs * phi;
        Axnorm = std::sqrt(Axnorm * Axnorm + tau * tau);
        phi = sn * phi;
    }
    void applyRightPrev()
    {
        AT("Right prev")
        if (iter > 2)
        {
            veplnl2 = veplnl;
            etal2 = etal;
            etal = eta;
            auto dlta_tmp = sr2 * vepln - cr2 * dlta;
            veplnl        = cr2 * vepln + sr2 * dlta;
            dlta = dlta_tmp;
            eta = sr2*gamma;
            gamma = -cr2*gamma;

        }
    }
    void applyRightCurr()
    {
        AT("Right curr")
        if (iter > 1)
        {
            givens2(gammal, dlta, cr1, sr1, gammal);
            vepln = sr1 * gamma;
            gamma = -cr1 * gamma;
        }
    }
    void updateXNorm()
    {
        AT("X norm")
        auto ul4 = ul3;
        ul3 = ul2;
        if (iter > 2)
            ul2 = (taul2 - etal2 * ul4 - veplnl2 * ul3) / gammal2;
        if (iter > 1)
            ul  = (taul  - etal  * ul3 - veplnl  * ul2) / gammal;

        auto xnorm_tmp = std::sqrt(xl2norm * xl2norm + ul2 * ul2 + ul * ul);
        if (std::abs(gamma) > std::numeric_limits<double>::min() && xnorm_tmp < maxXNorm)
        {
            u = (tau - eta * ul2 - vepln * ul) / gamma;
            if (std::sqrt(xnorm_tmp * xnorm_tmp + u * u) > maxXNorm)
            {
                u = 0;
                flag = MinresQLPStatus::XNORM;
            }
        }
        else
        {
            u = 0;
            flag = MinresQLPStatus::LSQ_NOCONV;
        }
        xl2norm = std::sqrt(xl2norm * xl2norm + ul2 * ul2);
        xnorm   = std::sqrt(xl2norm * xl2norm + ul * ul + u * u);
    }
    void minresStep()
    {
        {
        AT("Minres w-swap")
        wl2 = std::move(wl);
        wl = std::move(w);
        }
        {
        AT("Minres w-calc")
        w = (v - epln * wl2 - dlta_QLP * wl) / gamma_tmp;
        }
        {
        AT("Minres x-calc")
        if (xnorm < maxXNorm)
            x += tau * w;
        else
            flag = MinresQLPStatus::XNORM;
        }
    }
    void minresQLPStep()
    {
        AT("Minres QLP")
        ++QLPiter;
        if (QLPiter == 1)
        {
            if (iter > 1)
            {
                if (iter > 3)
                    wl2 = gammal3 * wl2 + veplnl2 * wl + etal * w;
                if (iter > 2)
                    wl  = gammal_QLP * wl + vepln_QLP * w;
                w = gamma_QLP * w;
                xl2 = x - wl * ul_QLP - w * u_QLP;
            }
        }
        switch (iter)
        {
        case 1:
            wl2 = std::move(wl);
#ifndef WITH_BLAS
            wl = v * sr1;
#else
            wl = v;
            cblas_dscal(wl.size(), sr1, &wl[0], 1);
#endif
#ifndef WITH_BLAS
            w  =-v * cr1;
#else
            w = v;
            cblas_dscal(w.size(), -cr1, &w[0], 1);
#endif
            break;
        case 2:
            wl2 = std::move(wl);
#ifndef WITH_BLAS
            wl  = w * cr1 + v * sr1;
#else
            wl = w;
            cblas_dscal(wl.size(), cr1, &wl[0], 1);
            cblas_daxpy(wl.size(), sr1, &v[0], 1, &wl[0], 1);
#endif
#ifndef WITH_BLAS
            w   = w * sr1 - v * cr1;
#else
            cblas_dscal(w.size(), sr1, &w[0], 1);
            cblas_daxpy(w.size(),-cr1, &v[0], 1, &w[0], 1);
#endif
            break;
        default:
            wl2 = std::move(wl);
            wl  = std::move(w);
#ifndef WITH_BLAS
            w   = wl2 * sr2 - v * cr2;
#else
            w = wl2;
            cblas_dscal(w.size(), sr2, &w[0], 1);
            cblas_daxpy(w.size(), -cr2, &v[0], 1, &w[0], 1);
#endif
#ifndef WITH_BLAS
            wl2 = wl2 * cr2 + v * sr2;
#else
            cblas_dscal(wl2.size(), cr2, &wl2[0], 1);
            cblas_daxpy(wl2.size(), sr2, &v[0], 1, &wl2[0], 1);

#endif
#ifndef WITH_BLAS
            v   = wl  * cr1 + w * sr1;
#else
            v = wl;
            cblas_dscal(v.size(), cr1, &v[0], 1);
            cblas_daxpy(v.size(), sr1, &w[0], 1, &v[0], 1);

#endif
#ifndef WITH_BLAS
            w   = wl  * sr1 - w * cr1;
#else
            cblas_dscal(w.size(), -cr1, &w[0], 1);
            cblas_daxpy(w.size(), sr1, &wl[0], 1, &w[0], 1);
#endif
            wl = std::move(v);
        }
#ifndef WITH_BLAS
        xl2 += wl2 * ul2;
#else
        cblas_daxpy(xl2.size(), ul2, &wl2[0], 1, &xl2[0], 1);
#endif
#ifndef WITH_BLAS
        x   = xl2 + wl  * ul   + w * u;
#else
        x = xl2;
        cblas_daxpy(x.size(), ul, &wl[0], 1, &x[0], 1);
        cblas_daxpy(x.size(), u , & w[0], 1, &x[0], 1);
#endif
    }
    void computeNextRight()
    {
        AT("Next right")
        auto gammal_tmp = gammal;
        givens2(gammal, eplnn, cr2, sr2, gammal);

        // Storing stuff for QLP
        gammal_QLP = gammal_tmp; vepln_QLP = vepln; gamma_QLP = gamma;
        ul_QLP    = ul;        u_QLP     = u;
    }
    void estimateNorms()
    {
        AT("Norms")
        auto abs_gamma = std::abs(gamma);
        Anorm = std::max(Anorm, std::max(pnorm, std::max(gammal, abs_gamma)));
        if (iter == 1)
        {
            gmin  = gamma;
            gminl = gmin;
        }
        else
        {
            auto gminl2 = gminl;
            gminl = gmin;
            gmin = std::min(gminl2, std::min(gammal, abs_gamma));
        }
        Acondl = Acond;
        Acond = Anorm / gmin;
        rnorml  = rnorm;
        relresl = relres;
        if (flag != MinresQLPStatus::LSQ_NOCONV)
            rnorm = phi;
        relres = rnorm / (Anorm * xnorm + beta1);
        auto rootl  = std::sqrt(gbar * gbar + dltan * dltan);
//            Arnorml= rnorml * rootl;
        relAresl = rootl / Anorm;
    }
    void checkExit()
    {
        AT("Exit")
        auto epsx = Anorm * xnorm * eps;
        if (flag == MinresQLPStatus::RUNNING || flag == MinresQLPStatus::LSQ_NOCONV)
        {
            auto t1 = 1.0 + relres,
                t2 = 1.0 + relAresl;
            do
            {
#define S(c, s) \
                if (c) \
                { \
                    flag = MinresQLPStatus::s; \
                    break; \
                }
                S(iter >= maxIt,     ITERATION)
                S(Acond >= ACondLim, ACOND)
                S(xnorm >= maxXNorm, XNORM)
                S(epsx >= beta1,     EIGEN)
                S(t2 <= 1.0,         MINLEN_EPS)
                S(t1 <= 1.0,         SOLVED_EPS)
                S(relAresl <= rtol,  MINLEN_RTOL)
                S(relres <= rtol,    SOLVED_RTOL)
#undef S
            } while (0);
        }

        if (flag == MinresQLPStatus::MINLEN_RTOL ||
            flag == MinresQLPStatus::MINLEN_EPS  ||
            flag == MinresQLPStatus::XNORM       ||
            flag == MinresQLPStatus::ACOND)
        {
            --iter;
            Acond = Acondl;
            rnorm = rnorml;
            relres = relresl;
        }
    }


    const M& A;
    const Vector &b;
    Vector &x;
    int N;
    Vector w, wl, wl2, xl2, r2, r3, r1, v;
    double Arnorm = 0.0, ynorm = 0.0;

    // Here we have some supplementary state
    MinresQLPStatus flag = MinresQLPStatus::RUNNING;
    int iter = 0, QLPiter = 0;
    double beta1 = 0.0,
         beta   =   0.0,   tau = 0.0,    taul = 0.0,  phi = 0.0,
        betan  = 0.0,  gmin = 0.0,      cs =-1.0,   sn = 0.0,
        cr1   =  -1.0,   sr1 = 0.0,     cr2 =-1.0,  sr2 = 0.0,
        dltan =   0.0, eplnn = 0.0,    gamma = 0.0, gammal= 0.0,
        gammal2=   0.0,   eta = 0.0,    etal = 0.0, etal2= 0.0,
        vepln =   0.0, veplnl= 0.0,  veplnl2= 0.0,  ul3 = 0.0,
            ul2 =   0.0,    ul =   0.0,     u = 0.0,
    gammal_QLP = 0.0, vepln_QLP = 0.0, gamma_QLP = 0.0,
    u_QLP = 0.0, ul_QLP = 0.0,
    gminl = 0.0, rnorm  = 0.0, xnorm = 0.0, xl2norm = 0.0, Axnorm = 0.0,
    Anorm = 0.0, Acond = 1.0,
    relres = 0.0,

    alpha = 0.0, pnorm = 0.0,
    dlta, epln, gbar, dlta_QLP,
    gammal3, gamma_tmp, taul2,
    Acondl, rnorml, relresl, relAresl;

    void givens2(double a, double b, double &c, double &s, double &d)
    {
        if (b == 0.0)
        {
            c = a >= 0.0 ? 1.0 : -1.0;
            s = 0.0;
            d = std::abs(a);
            return;
        }
        if (a == 0.0)
        {
            c = 0.0;
            s = b > 0.0 ? 1.0 : -1.0;
            d = std::abs(b);
            return;
        }
        if (std::abs(b) > std::abs(a))
        {
            auto t = a / b;
            s = (b > 0.0 ? 1.0 : -1.0) / std::sqrt(1.0 + t * t);
            c = s * t;
            d = b / s;
            return;
        }
        auto t = b / a;
        c = (a > 0.0 ? 1.0 : -1.0) / std::sqrt(1.0 + t * t);
        s = c * t;
        d = a / c;
    }
};
};
