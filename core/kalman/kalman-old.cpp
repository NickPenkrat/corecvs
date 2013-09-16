/*************************************************************************/
/*                                                                       */
/*  Kalman Filter                                                        */
/*                                                                       */
/*  Author: Schick (Wuensche,Christians)                      31.7.89    */
/*          Stein                                             20.1.94    */
/*                o extension for control vector Bu (B linear)           */
/*                o extension for multiple kalman filters by introducing */
/*                  kafi_id as a identifier                              */
/*                                                                       */
/*          Strasser                                         29.11.94    */
/*                o added code for effective matrices                    */
/*                o added workspace parameter for internal needed memory */
/*                  (parallel use in threads, no limits if used in lib ) */
/*                o added functions to init various structs defined in   */
/*                  header file                                          */
/*  Literature:                                                          */
/*                                                                       */
/*************************************************************************/

#include <math.h>
#include <stddef.h>
#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include "kalman.h"
namespace corecvs {


/* remove comments to use save(r) malloc/free */
#define OSH_useSaveMem
//#include "savemem.h" //east

DyVector::DyVector(int ymax)
{
    index = (int*) malloc(ymax * sizeof(int));
    dy = (tFloat*) malloc(ymax * sizeof(tFloat));

    if (index == 0 || dy == 0)
    {
        free(index);
        free(dy);
        maxNumber = 0;
    }

    maxNumber = ymax;

    for (int i = 0; i < ymax; i++)
        index[i] = i;

    number = 0; /*Add East*/
}

DyVector::~DyVector()
{
    free(index);
    free(dy);

}

KalmanWorkspace::KalmanWorkspace(int nmax)
{
    fv0 = (tFloat*) malloc(nmax * nmax * sizeof(tFloat));
    fv1 = (tFloat*) malloc(nmax * nmax * sizeof(tFloat));
    if (nmax <= 1)
        fv2 = (tFloat*) malloc(2 * nmax * sizeof(tFloat));
    else
        fv2 = (tFloat*) malloc(nmax * nmax * sizeof(tFloat));
    fv3 = (tFloat*) malloc(nmax * sizeof(tFloat));
    iv0 = (int*) malloc(2 * nmax * sizeof(int));
    iv1 = (int*) malloc(nmax * sizeof(int));

    if (fv0 == 0 || fv1 == 0 || fv2 == 0 || fv3 == 0 || iv0 == 0 || iv1 == 0){
        nmax = 0;
        nphimax = 0;
        free(fv0);
        free(fv1);
        free(fv2);
        free(fv3);
        free(iv0);
        free(iv1);
    }
    else{
        nmax    = nmax;
        nphimax = nmax * nmax;
    }

}

KalmanWorkspace::~KalmanWorkspace()
{
    free(fv0);
    free(fv1);
    free(fv2);
    free(fv3);
    free(iv0);
    free(iv1);
}



/*************************************************************************/
/*                                                                       */
/*  Time-Update nach Bierman UD-faktorisiert, fuer schwach besetzte      */
/*  Время обновления Бирман UD факторизации, для разреженных             */
/*  Time to update Bierman UD factorization, for sparse                  */
/*                                                                       */
/*  Transitionsmatrizen                                                  */
/*                                                                       */
/*  umgeschrieben aus Fortran in C                                       */
/*                                                                       */
/*  Autor : Schick (Wuensche,Christians)                      31.7.89    */
/*          Aenderungen: Stein                                20.1.94    */
/*                                                                       */
/*************************************************************************/
/*
tFloat *diagonalMatrix,       Diagonale von P = U D U~

tFloat* upperTriangleMatrix,      Upper Triangle von P = U D U~

COOSparseMatrix* transitionMatrix,    Transitions-Matrix

tFloat *errorCovMatrix,        Systemfehler-Kovarianz-Matrix
                          Системная ошибка ковариационной матрицы
                          System error covariance matrix

int stateVectorLength,                   Laenge des Zustandsvektor
                          Длина вектора состояния
                          Length of the state vector

KalmanWorkspace *workspace   Structure with 4 float arrays, 2 int arrays
                          and 2 int numers
*/
static void
        kfudt(tFloat *diagonalMatrix,
              tFloat* upperTriangleMatrix,
              COOSparseMatrix* transitionMatrix,
              tFloat *errorCovMatrix,
              int stateVectorLength,
              KalmanWorkspace *workspace)
{
    /* Gedaechtnis ...
        тут переменные типа */

    int i,currentRow,currentColumn,iw0,jj,kol,j,jw0,jpn;
    tFloat        currentValue,sum,dinv;

    tFloat *w  = workspace->fv0; // [NMAX*NMAX]
    tFloat *v  = workspace->fv2; // [2*NMAX]
    tFloat *diagonalElements = workspace->fv1; // [NPHIMAX]
    int *jj0 = workspace->iv0; // [NPHIMAX]   /* 20 > nx */
    int *icw = workspace->iv1; // [NPHIMAX]

    for(i = 0; i < workspace->nphimax; i++)
        w[i]=0.0;

    /* Setze Indices
        Set indices */

    jj0[0] = 0;
    jj0[1] = 1;
    for(i = 1; i < stateVectorLength-1; i++)
        jj0[i+1] = jj0[i] + i;
    for(i = 0; i < stateVectorLength; i++)
        icw[i] = stateVectorLength * i;

    /* Sichere die Diagonal-Elemente
        Secure the diagonal elements */

    for(i = 0; i < stateVectorLength; i++) {
        diagonalElements[i] = diagonalMatrix[i];
        v[stateVectorLength+i] = 0.0;
    }

    /* Algorithmus fuer PHI[*,*] * U[*,*] bei schwach besetzten Transitions- */
    /* matrizen ( nach "Bierman's CCG-Kurs", p.55

        Algorithm for PHI [*,*] * U [*,*] with sparse transition matrices */

    for (unsigned k = 0; k < transitionMatrix->elements.size(); k++) {
        currentRow = transitionMatrix->elements[k].row + 1;
        currentColumn = transitionMatrix->elements[k].column + 1;
        currentValue = transitionMatrix->elements[k].value;

        /* row of Phi-matrix = col of W-matrix */

        iw0 = icw[currentRow - 1];
        w[iw0 + currentColumn - 1] += currentValue;

        if (currentColumn < stateVectorLength) {
            jj = jj0[currentColumn] - 1;

            /* Hole Anfangswert ( Stelle der ersten Zeile von U im Vektor UV )  */
            /* aus der Tabelle jj0

               Hole initial value (in the first line of the vector U UV) from table jj0 */

            for (kol = currentColumn; kol < stateVectorLength; kol++) {
                jj += kol - 1;
                w[iw0 + kol] += currentValue * upperTriangleMatrix[jj];
            }
        }
    }

    /* it is assumed in the following, that W[i,n+j] = BNOISE[i,j]      */

    for (j = stateVectorLength - 1; j > -1; j--) {
        sum = 0.0;
        jw0 = icw[j];
        for (i = 0; i < stateVectorLength; i++) {
            v[i] = w[jw0 + i] * diagonalElements[i];
            sum += v[i] * w[jw0 + i];
        }
        sum += errorCovMatrix[j];
        diagonalMatrix[j] = sum;

        /* Loesche letzten Eintrag aus j+1 in V, trage Q[j] in V[*nx+j] ein

            Loesche last entry of j +1 in V, wearing Q [j] into V [* nx + j] */

        jpn = j + stateVectorLength;
        if (j < stateVectorLength - 1)
            v[jpn + 1] = 0.0;
        v[jpn] = errorCovMatrix[j];
        if (j == 0)
            return;

        jj = jj0[j] - 1;
        if (sum == 0.0)
            for (i = 0; i < j; i++)
                upperTriangleMatrix[jj + i] = 0.0;
        else {
            dinv = (tFloat) (1.0 / sum);
            for (i = 0; i < j; i++) {
                sum = 0.0;
                iw0 = icw[i];
                for (int k = 0; k < stateVectorLength; k++)
                    sum += w[iw0 + k] * v[k];
                sum *= dinv;
                upperTriangleMatrix[jj + i] = sum;
                for (int k = 0; k < stateVectorLength; k++)
                    w[iw0 + k] -= sum * w[jw0 + k];
            }
        }
    }
}

/*************************************************************************/
/*                                                                       */
/*  Praediktion der Kovarianzmatrix P = UDU~ ueber P = F P F~ + Q        */
/*  Прогнозирование ковариационной матрицs P = UDU~ где P = F P F~ + Q   */
/*  Prediction of the covariance matrix P = UDU~ where P = F P F~ + Q    */
/*                                                                       */
/*  Autor : Schick                                            22.2.90    */
/*          Aenderungen: Stein                                20.1.94    */
/*                                                                       */
/*************************************************************************/
/*
Klm_S_varimat *b_p,           P = U D U~

tFloat i_q[],         Float Array
                            Systemfehler-Kovarianz-Matrix
                            Системная ошибка ковариационной матрицы
                            System error covariance matrix

Klm_S_eff_matrix *i_f,      Structure witn 3 arrays (2 int and 1 float)
                            Funktional-Matrix F
                            Функциональная матрица F
                            Functional matrix F

int i_nx,                     Anzahl der Zustandsgroessen
                            Количество переменных состояния
                            Number of state variables

KalmanWorkspace *b_ws    Structure with 4 float arrays, 2 int arrays
                            and 2 int numers
*/
void KalmanWorkspace::calculatePrediction(UdutMatrix *b_p, tFloat i_q[], COOSparseMatrix *i_f, int i_nx)
{
    kfudt(b_p->d, b_p->u, i_f, i_q, i_nx, this);
}

/*************************************************************************/
/*                                                                       */
/*  Measurement-Update nach Bierman UD-faktorisiert                      */
/*                                                                       */
/*  umgeschrieben aus Fortran in C                                       */
/*                                                                       */
/*  Autor : Schick (Wuensche,Christians)                    31.7.89      */
/*          Aenderungen: Stein                              20.1.94      */
/*                                                                       */
/*************************************************************************/
/*Example: kfudm(i_nx, p->u, p->d, &(dy_help[in]), &c[i_nx * in], &r[in], dxseq, &a, b,b_ws);*/

/*
int stateVariablesNumber,                   Anzahl der Zustandsgroessen
                            Количество переменных состояния
                            Number of state variables

tFloat *upperTriangleMatrix,        Float Array
                            Upper triangle U     P = U D U~

tFloat *diagonalMatrix,        Float Array
                            Diagonale D          P = U D U~

tFloat *jDimension,      Float Number
                            j-ter Messwert
                            j - ое измерение
                            J-th dimension

tFloat *jColumn,       Float Array
                            j-te Zeile von C
                            j-я строка с
                            j-th row of с

tFloat *jDiagonalElement,       Float Number
                            j-tes Diagonalelement von R
                            J-й диагональный элемент R
                            J-th diagonal element of R

tFloat *stateVaiablesCorrection,       Float Array
                            Korrektur der Zustandsgroessen
                            Коррекция переменных состояния
                            Correction of the state variables

tFloat *innovationCovariance,        Float Number
                            Innovations-Kovarianz
                            Инновации ковариации
                            Innovation covariance

tFloat *gainVector,        Float Array
                            Kalman-Filter Gain Vektor
                            вектор усиления Фильтра Калмана
                            Kalman filter gain vector

KafiWorkspace *workspace         Structure with 4 float arrays, 2 int arrays
                            and 2 int numers

*/
static void
        kfudm(int stateVariablesNumber
              , tFloat *upperTriangleMatrix
              , tFloat *diagonalMatrix
              , tFloat *jDimension
              , tFloat *jColumn
              , tFloat *jDiagonalElement
              , tFloat *stateVaiablesCorrection /*b_ws->fv2*/
              , tFloat *innovationCovariance
              , tFloat *gainVector /* b_ws->fv1 */
              , KalmanWorkspace *workspace
              )
{
    tFloat        *f;/*[NMAX]*/            /* 20 > nx */
    int jj = ((stateVariablesNumber-1) * stateVariablesNumber) / 2;
    int j,k;
    tFloat alfa,beta,gamma,delta,lambda;
    tFloat sum,z;

    f = workspace->fv3;

    /* compute first B = D U~C~ and F~ = C U */
    for(j = stateVariablesNumber - 1; j > 0; j--)
    {
        alfa = jColumn[j];
        jj  -= j;
        for(k = 0; k < j; k++)
        {
            alfa += upperTriangleMatrix[jj+k]*jColumn[k];
        }
        f[j] = alfa;
        gainVector[j] = diagonalMatrix[j] * alfa;
    }

    gainVector[0] = diagonalMatrix[0] * jColumn[0];
    f[0] = jColumn[0];

    /* next: update U and D matrix, compute gains */
    alfa = *jDiagonalElement + gainVector[0]*f[0];
    if(alfa > 0.0) {
        gamma = (tFloat)(1.0/alfa);
        if(gainVector[0] != 0.0) diagonalMatrix[0] *= *jDiagonalElement * gamma;
    }
    else {
        gamma = 0.0;
        if(gainVector[0] != 0.0) diagonalMatrix[0] = 0.0;
    }

    jj = -1;
    for(j = 1; j < stateVariablesNumber; j++) {
        beta  = alfa;
        delta = gainVector[j];
        alfa += delta * f[j];

        if((alfa != 0.0) && (j < stateVariablesNumber)) {
            lambda = -f[j] * gamma;
            for(k = 0; k < j; k++) {
                jj++;
                sum   = upperTriangleMatrix[jj];
                upperTriangleMatrix[jj] = sum + lambda * gainVector[k];
                gainVector[k] += delta * sum;
            }
            gamma   = (tFloat)(1.0 / alfa);
            diagonalMatrix[j] *= beta * gamma;
        }
    }

    z = *jDimension * gamma;

    for(j = 0; j < stateVariablesNumber; j++)
    {
        stateVaiablesCorrection[j] = gainVector[j] * z;
    }

    *innovationCovariance = alfa;
}


/* ************************************************************************/
/*                                                                       */
/*  Innovation von Kovarianz-Matrix P = UDU~ und Zustandsvektor          */
/*                                                                       */
/*  mit update ueber alle Merkmale                                       */
/*                                                                       */
/*  Autor : Schick                                           22.2.90     */
/*          Aenderungen: Stein                               20.1.94     */
/*                                                                       */
/* ************************************************************************/
/*
tFloat        i_r[];            Float Array
                            Messfehler-Kovarianz-Matrix
                            Погрешность измерения ковариационной матрицей
                            Measurement error covariance matrix

varimat        *b_p;                      Structure with 2 Float Array (d & u)
                            P = U D U~

tFloat *i_c;        Float Array
                            Jakobi -Matrix Mess-/Schaetzgroesse
                       Матрица Якоби
                       Jacobi matrix

y_vek *b_y;                                          Structure with 2 arrays (int index and float dy)
                       and 2 int values
                       d,dy = y(Mess) - y(vermutet)
                        Mess – может измеряемый (may be measured)
                            Vermutet – предполагаемый (expected)

int                i_nx;                  Laenge des Schaetzvektors
                         Длина Schaetz вектора
                       Length of Schaetz vector

tFloat i_xs[];      Float Array
                            Schaetzvektor geschaetzt
                           Оценка для Schaetz вектора
                       The estimate for the vector Schaetz

tFloat        o_xd[];            Float Array
                            verbesserter Schaetzvektor
                            улучшение  для Schaetz вектора
*/
void KalmanWorkspace::calculateInnovation(tFloat i_r[], UdutMatrix *b_p, Matrix *measurementMatrix, DyVector *b_y,
                      int i_nx, tFloat i_xs[], tFloat o_xd[])

/*
  int           kafi_id; identifier
  tFloat        i_r[];             Messfehler-Kovarianz-Matrix
  varimat        *p;             P = U D U~
  tFloat        *i_c;             Jakobi-Matrix Mess-/Schaetzgroesse
  y_vek                *y;                 d,dy = y(Mess) - y(vermutet)
  int                nx;             Laenge des Schaetzvektors
  tFloat        i_xs[];             Schaetzvektor geschaetzt
  tFloat        xd[];             verbesserter Schaetzvektor           */

{
    if(i_r && b_p && measurementMatrix && b_y && i_xs && o_xd && this)
    {
        tFloat        *dx/*[NMAX]*/;                  /* Korrektur des Schaetzvektors i_xs()    */
        tFloat        a=0.0;                         /* Innovations-Kovarianz                */
        tFloat        *b/*[NMAX]*/;                 /* Kalman-Filter Gain Vektor            */
        tFloat        *dxseq/*[NMAX]*/;                 /* 10 > nx                              */

        tFloat        *dy_help;
        /* tFloat        *cs;  - not used */
        /* tFloat        *rs;  - not used */
        int                ix,iy;
        int in, inp1;

        dx    = fv0;
        b     = fv1;
        dxseq = fv2;

        dy_help = &(b_y->dy[0]);


        for(ix = 0; ix < i_nx; ix++) dx[ix] = 0.0;

        for(iy = 0; iy < b_y->number; iy++)
        {
            in = b_y->index[iy];
            /*kfudm(int i_nx, tFloat *b_u, tFloat *b_d,
        tFloat *i_dyj, tFloat *i_cj, tFloat *i_rj,
        tFloat *o_dx  , tFloat *o_a, tFloat *o_b,
        KafiWorkspace *this)*/
            kfudm(i_nx, b_p->u, b_p->d, &(dy_help[in]), &measurementMatrix->a(i_nx * in), &i_r[in], dxseq, &a, b, this);
            for(ix = 0; ix < i_nx; ix++)
            {
                dx[ix] += dxseq[ix];
                if(iy < b_y->number - 1 )
                {
                    inp1 = b_y->index[iy+1];
                    dy_help[inp1] -= measurementMatrix->a(i_nx * inp1 + ix) * dx[ix];
                }
            }
        }

        for(ix = 0; ix < i_nx; ix++) o_xd[ix] = i_xs[ix] + dx[ix];
    }

}

/*************************************************************************/
/*                                                                       */
/*  Vorhersage des Schaetzvektors xs aus xd ueber ts Zyklen              */
/*  Прогнозирование Schaetzvektors от XD TS XS о циклах                  */
/*  Prediction of Schaetzvektors from xd xs ts about cycles              */
/*                                                                       */
/*  Autor : Schick                                           22.2.90     */
/*          Aenderungen: Stein - Einbau B und u              20.1.94     */
/*                       NUR fuer ts = 1!!!                              */
/*                                                                       */
/*************************************************************************/
/*
 Klm_S_eff_matrix *transition,   Transitionsmatrix
                                 Матрица перехода
                                 Transition Matrix

 int        i_ts,                 Anzahl der Vorhersagezyklen
                                  Количество циклов Прогнозирования
                                  Number of cycles Forecast

 COOSparseMatrix *B,             Steuermatrix
                                  Матрица управления
                                  Control matrix

 tFloat     i_u[],        Steuervektor
                                  Вектор управления
                                  Control vector

 int        i_nx,                 Laenge des Schaetzvektors
                                  Длина Schaetzvektors
                                  Length of Schaetzvektors

 tFloat     i_xd[],      verbesserter Schaetzvektor
                                 улучшение Schaetzvektor
                                 improved Schaetzvektor

 tFloat     o_xs[],      vorhergesagter Schaetzvektor
                                 предсказанный Schaetzvektor
                                 predicted Schaetzvektor
*/
void KalmanWorkspace::modelStateTransition(
        COOSparseMatrix *transitionMatrix,
        int        i_ts,
        COOSparseMatrix *controlMatrix,
        tFloat     i_u[],
        int        i_nx,
        tFloat     i_xd[],
        tFloat     o_xs[])
{
    if (!transitionMatrix || !i_xd || !o_xs || !this)
        return;

    int i, j, ir, ic, t;
    tFloat *xh/*[NMAX]*/;

    xh = fv0;

    for (i = 0; i < i_nx; i++)
        xh[i] = i_xd[i];

    for (t = 0; t < i_ts; t++) {

        /* TODO: Move two blocks below to separate function in the sparseMatrix class */
        for (i = 0; i < i_nx; i++)
            o_xs[i] = 0.0;

        j = transitionMatrix->elements.size();

        for (i = 0; i < j; i++) {
            ir = transitionMatrix->elements[i].row;
            ic = transitionMatrix->elements[i].column;
            o_xs[ir] += transitionMatrix->elements[i].value * xh[ic];
        }

        /*TODO: Think about uniting B and u data structures*/
        if (controlMatrix != NULL) {

            /* Bu */
            j = controlMatrix->elements.size();

            for (i = 0; i < j; i++) {
                ir = controlMatrix->elements[i].row;
                ic = controlMatrix->elements[i].column;
                o_xs[ir] += controlMatrix->elements[i].value * i_u[ic];
            }
        }

        for (i = 0; i < i_nx; i++)
            xh[i] = o_xs[i];
    }
}

} //namespace corecvs

