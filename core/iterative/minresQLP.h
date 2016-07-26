#include "vector.h"

namespace corecvs
{
namespace Iterative
{
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

template<typename M>
void minresQlp(const M &A, const Vector &b, Vector &x)
{
    int itn = 0;
    double Arnorm = 0.0, ynorm = 0.0;
    bool done = false;
    int N = (int)b.size();

    auto r2 = b, r3 = b, r1 = Vector(N);
    auto beta1 = !r2;

    int flag0 = -2, iter = 0, QLPiter = 0;
    auto flag = flag0;
    auto beta   =   0.0,   tau = 0.0,    taul = 0.0,  phi = beta1,
         betan  = beta1,  gmin = 0.0,      cs =-1.0,   sn = 0.0,
          cr1   =  -1.0,   sr1 = 0.0,     cr2 =-1.0,  sr2 = 0.0,
          dltan =   0.0, eplnn = 0.0,    gamma = 0.0, gammal= 0.0,
          gammal2=   0.0,   eta = 0.0,    etal = 0.0, etal2= 0.0,
          vepln =   0.0, veplnl= 0.0,  veplnl2= 0.0,  ul3 = 0.0,
            ul2 =   0.0,    ul =   0.0,     u = 0.0,
      gammal_QLP = 0.0, vepln_QLP = 0.0, gamma_QLP = 0.0,
      u_QLP = 0.0, ul_QLP = 0.0,
      gminl = 0.0;
    auto rnorm  = betan, xnorm = 0.0, xl2norm = 0.0, Axnorm = 0.0;
    auto Anorm = 0.0, Acond = 1.0;
    auto relres = rnorm / (beta1 + std::numeric_limits<double>::min());
    auto eps = 2.3e-16;
    x = Vector(N);
    Vector w(N), wl(N), wl2(N), xl2(N);

    int maxIt = N;
    double rtol = 1e-16, maxXNorm = 1e100, ACondLim = 1e100, TranCond = 1e7;

    std::cout << "How does it works?" << std::endl;
    if (beta1 == 0.0)
        return;

    std::cout << "It starts with iterations " << std::endl;
        std::cout << /*x << " : " <<*/ !(A * x - b) /*<< std::endl*/;
    while (flag == flag0 && iter < maxIt)
    {
        std::cout << "*" << std::flush;
        ++iter;
        auto betal = beta;
        beta  = betan;

        auto v = r3 / beta;
        r3 = A * v;

        if (iter > 1)
            r3 = r3 - (beta / betal) * r1;

        auto alpha = r3 & v;
        r3 = r3 - (alpha / beta) * r2;
        r1 = r2;
        r2 = r3;

        betan = !r3;
        if (iter == 1 && betan == 0.0)
        {
            if (alpha == 0.0)
            {
                flag = 0;
				std::cout << "x = 0 is a solution" << std::endl;
                break;
            }
            flag = -1;
            x = b / alpha;
			std::cout << "b and x are eigen" << std::endl;
            break;
        }

        auto pnorm = std::sqrt(betal * betal + alpha * alpha + betan * betan);

        // Prev left rotation
        auto dbar = dltan;
        auto dlta = cs * dbar + sn * alpha;
        auto epln = eplnn;
        auto gbar = sn * dbar - cs * alpha;
        eplnn = sn * betan;
        dltan = -cs * betan;
        auto dlta_QLP = dlta;

        // Current left rotation
        auto gammal3 = gammal2;
        gammal2 = gammal;
        gammal = gamma;
        givens2(gbar, betan, cs, sn, gamma);
        auto gamma_tmp = gamma;
        auto taul2 = taul;
        taul = tau;
        tau = cs * phi;
        Axnorm = std::sqrt(Axnorm * Axnorm + tau * tau);
        phi = sn * phi;

        // Previous right rotation
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

        // Current right rotation
        if (iter > 1)
        {
            givens2(gammal, dlta, cr1, sr1, gammal);
            vepln = sr1 * gamma;
            gamma = -cr1 * gamma;
        }

        // Updating xnorm
        auto xnorml = xnorm;
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
                flag = 6;
            }
        }
        else
        {
            u = 0;
            flag = 9;
        }
        xl2norm = std::sqrt(xl2norm * xl2norm + ul2 * ul2);
        xnorm   = std::sqrt(xl2norm * xl2norm + ul * ul + u * u);

        // Updating w
        if (Acond < TranCond && flag != flag0 && QLPiter == 0)
        {
            // Here we go minres
            wl2 = wl;
            wl = w;
            w = (v - epln * wl2 - dlta_QLP * wl) / gamma_tmp;
            if (xnorm < maxXNorm)
                x += tau * w;
            else
                flag = 6;
        }
        else
        {
            // Here we go minres-qlp
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
                wl2 = wl;
                wl = v * sr1;
                w  =-v * cr1;
                break;
            case 2:
                wl2 = wl;
                wl  = w * cr1 + v * sr1;
                w   = w * sr1 - v * cr1;
                break;
            default:
                wl2 = wl;
                wl  = w;
                w   = wl2 * sr2 - v * cr2;
                wl2 = wl2 * cr2 + v * sr2;
                v   = wl  * cr1 + w * sr1;
                w   = wl  * sr1 - w * cr1;
                wl = v;
            }
            xl2 = xl2 + wl2 * ul2;
            x   = xl2 + wl  * ul   + w * u;
        }

        // Computing next right rotation
        auto gammal_tmp = gammal;
        givens2(gammal, eplnn, cr2, sr2, gammal);

        // Storing stuff for QLP
        gammal_QLP = gammal_tmp; vepln_QLP = vepln; gamma_QLP = gamma;
        ul_QLP    = ul;        u_QLP     = u;

        // Estimating norms
        auto abs_gamma = std::abs(gamma), Anorml = Anorm;
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
        auto Acondl = Acond;
        Acond = Anorm / gmin;
        auto rnorml  = rnorm;
        auto relresl = relres;
        if (flag != 9)
            rnorm = phi;
        relres = rnorm / (Anorm * xnorm + beta1);
        auto rootl  = std::sqrt(gbar * gbar + dltan * dltan),
             Arnorml= rnorml * rootl;
        auto relAresl = rootl / Anorm;

        // Are we ready?
        auto epsx = Anorm * xnorm * eps;
        if (flag == flag0 || flag == 9)
        {
            auto t1 = 1.0 + relres,
                 t2 = 1.0 + relAresl;
            do
            {
                if (iter >= maxIt)
                {
                    flag = 8;
                    std::cout << "ITERATION LIMIT" << std::endl;
                    break;
                }
                if (Acond >= ACondLim)
                {
                    flag = 7;
                    std::cout << "A-CONDITION LIMIT" << std::endl;
                    break;
                }
                if (xnorm >= maxXNorm)
                {
                    flag = 6;
                    std::cout << "X-NORM LIMIT" << std::endl;
                    break;
                }
                if (epsx >= beta1)
                {
                    flag = 5;
                    std::cout << "X CONVERGED TO EIGEN" << std::endl;
                    break;
                }
                if (t2 <= 1.0)
                {
                    flag = 4;
                    std::cout << "X IS MIN-LENGTH LSQ (EPS)" << std::endl;
                    break;
                }
                if (t1 <= 1.0)
                {
                    flag = 3;
                    std::cout << "SOLVED (EPS)" << std::endl;
                    break;
                }
                if (relAresl <= rtol)
                {
                    flag = 2;
                    std::cout << "X IS MIN-LENGTH LSQ (RTOL)" << std::endl;
                    break;
                }
                if (relres <= rtol)
                {
                    flag = 1;
                    std::cout << "SOLVED (RTOL)" << std::endl;
                    break;
                }
            } while (0);
        }

        if (flag == 2 || flag == 4 || flag == 6 || flag == 7)
        {
            --iter;
            Acond = Acondl;
            rnorm = rnorml;
            relres = relresl;
        }
        std::cout << /*x << " : " <<*/ !(A * x - b) /*<< std::endl*/;
    }
		std::cout << "IT TOOK " << iter << " ITERATIONS" << std::endl;
        std::cout << std::endl;



}
};
};
