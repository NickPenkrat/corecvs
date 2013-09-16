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


DyVector::DyVector(int ymax)
{
    index = new int[ymax];
    dy = new double[ymax];

    if (index == 0 || dy == 0)
    {
        free(index);
        free(dy);
        maxNumber = 0;
    }

    maxNumber = ymax;

    for (int i = 0; i < ymax; i++)
        index[i] = i;
    number = 0;
}

DyVector::~DyVector()
{
    delete[] index;
    delete[] dy;

}

KalmanFilter::KalmanFilter(int stateSize, double stateNoise)
{
    Matrix *I = new Matrix(stateSize, stateSize, 1.0);
    F = new COOSparseMatrix(I);

    Q = new Matrix(1, stateSize);
    for (int i = 0; i < stateSize; i++)
        Q->a(0,i) = stateNoise;

    P = new UDUTDecomposed(I);
    delete I;


    oldState       = new Matrix(1, stateSize);
    predictedState = new Matrix(1, stateSize);
    currentState   = new Matrix(1, stateSize);

    nmax = stateSize;

    fv0 = (double*) malloc(nmax * nmax * sizeof(double));
    fv1 = (double*) malloc(nmax * nmax * sizeof(double));
    if (nmax <= 1)
        fv2 = (double*) malloc(2 * nmax * sizeof(double));
    else
        fv2 = (double*) malloc(nmax * nmax * sizeof(double));
    fv3 = (double*) malloc(nmax * sizeof(double));
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

KalmanFilter::~KalmanFilter()
{
    free(fv0);
    free(fv1);
    free(fv2);
    free(fv3);
    free(iv0);
    free(iv1);


    delete F;
    delete Q;
    delete P;

    delete oldState;
    delete predictedState;
    delete currentState;
}



/**
 *
 *  Time-Update nach Bierman UD-faktorisiert, fuer schwach besetzte
 *
 *  Transitionsmatrizen
 *
 *  umgeschrieben aus Fortran in C
 *
 *  Autor : Schick (Wuensche,Christians)                      31.07.89
 *          Aenderungen: Stein                                20.01.94
 *          Pimenov                                           10.10.11
 *
 * \param diagonalMatrix
 *     Matrix D form P = U D U^T decomposition
 * \param upperTriangleMatrix
 *     Matrix U form P = U D U^T decomposition
 * \param transitionMatrix
 *     State transition matrix
 * \param errorCovMatri
 *     System error covariance matrix
 * \param stateVectorLength,
 *     Length of the state vector
 * \param workspace
 *     Structure with 4 float arrays, 2 int arrays and 2 int numers
 **/
static void
        kfudt(double *diagonalMatrix,
              double* upperTriangleMatrix,
              COOSparseMatrix* transitionMatrix,
              double *errorCovMatrix,
              int stateVectorLength,
              KalmanFilter *workspace)
{
    /* Gedaechtnis ...*/

    int i,currentRow,currentColumn,iw0,jj,kol,j,jw0,jpn;
    double        currentValue,sum,dinv;

    double *w  = workspace->fv0; // [NMAX*NMAX]
    double *v  = workspace->fv2; // [2*NMAX]
    double *diagonalElements = workspace->fv1; // [NPHIMAX]

    /*These would be used as two index arrays*/
    int *jj0 = workspace->iv0; // [NPHIMAX]
    int *icw = workspace->iv1; // [NPHIMAX]

    for(i = 0; i < workspace->nphimax; i++)
        w[i] = 0.0;

    /* Set indices lookup arrays for upper triangle matrix */
    jj0[0] = 0;
    jj0[1] = 1;
    for(i = 1; i < stateVectorLength - 1; i++)
        jj0[i+1] = jj0[i] + i;

    /* Set indices lookup arrays for rectangular matrix */
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
            dinv = (double) (1.0 / sum);
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

/**
 *  Prediction of the covariance matrix \f$P = UDU^T\f$
 *  where \f$P_{k} = F_{k-1} P_{k-1} F_{k-1}^T + Q_{k-1} \f$
 *
 *  Autor  : Schick           22.2.90
 *  Changes: Stein            20.1.94
 *  Changes: A Pimenov        17.2.10
 *
 * \param P  covariance matrix \f$P\f$ in \f$UDU^T\f$ decomposed form
 * \param Q  System error covariance diagonal matrix
 * \param F  Functional matrix F
*/
void KalmanFilter::calculatePrediction(UDUTDecomposed *P, Matrix *Q, COOSparseMatrix *F)
{
    /* TODO: Make a deep check of the passed parameter */
    kfudt(P->diagonal->element, P->upper->elements, F, &(Q->a(0,0)), P->getSize(), this);
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
                            Number of state variables

double *upperTriangleMatrix,        Float Array
                            Upper triangle U     P = U D U~

double *diagonalMatrix,        Float Array
                            Diagonale D          P = U D U~

double *jDimension,      Float Number
                            j-ter Messwert
                            J-th dimension

double *jColumn,       Float Array
                            j-te Zeile von C
                            j-th row of с

double *jDiagonalElement,       Float Number
                            j-tes Diagonalelement von R
                            J-th diagonal element of R

double *stateVaiablesCorrection,       Float Array
                            Korrektur der Zustandsgroessen
                            Correction of the state variables

double *innovationCovariance,        Float Number
                            Innovations-Kovarianz
                            Innovation covariance

double *gainVector,        Float Array
                            Kalman-Filter Gain Vektor
                            Kalman filter gain vector

KafiWorkspace *workspace         Structure with 4 float arrays, 2 int arrays
                            and 2 int numers

*/
static void
        kfudm(int stateVariablesNumber
              , double *upperTriangleMatrix
              , double *diagonalMatrix
              , double *jDimension
              , double *jColumn
              , double *jDiagonalElement
              , double *stateVaiablesCorrection /*b_ws->fv2*/
              , double *innovationCovariance
              , double *gainVector /* b_ws->fv1 */
              , KalmanFilter *workspace
              )
{
    double        *f;/*[NMAX]*/            /* 20 > nx */
    int jj = ((stateVariablesNumber-1) * stateVariablesNumber) / 2;
    int j,k;
    double alfa,beta,gamma,delta,lambda;
    double sum,z;

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
        gamma = (double)(1.0/alfa);
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
            gamma   = (double)(1.0 / alfa);
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


/**
 *
 *
 *
 *
 * \param R                      Measurement noise vector
 * \param P                      State vector Covariance matrix
 * \param H                      Measurement Jacobian matrix
 * \param y                      Measurement misfit
 * \param stateVector            Measurement state vector
 * \param stateVectorInnovation  Measurement state innovated vector
 */
void KalmanFilter::calculateInnovation(
        Matrix         *R,
        UDUTDecomposed *P,
        Matrix         *H,
        DyVector       *y,
        Matrix         *stateVector,
        Matrix         *stateVectorInnovation)
{
    ASSERT_TRUE(R, "Measurement noise vector is null\n");
    ASSERT_TRUE(P, "State vector Covariance matrix is null\n");
    ASSERT_TRUE(H, "Measurement Jacobian matrix is null\n");
    ASSERT_TRUE(y, "Measurement misfit is null\n");
    ASSERT_TRUE(stateVector, "Measurement state vector is null\n");
    ASSERT_TRUE(stateVectorInnovation, "Measurement state innovated vector is null\n");

    ASSERT_TRUE (stateVector->w == stateVectorInnovation->w, "state and state innovated should have the same size");


    double        a = 0.0;                         /* Innovations-Kovarianz                */

    Matrix *gainVector    = new Matrix(1, stateVector->w);
    Matrix *corrector     = new Matrix(1, stateVector->w);
    Matrix *correctorStep = new Matrix(1, stateVector->w);


    for(int iy = 0; iy < y->number; iy++)
    {
        int in = y->index[iy];
        kfudm(
                stateVector->w,
                P->upper->elements,
                P->diagonal->element,
                &(y->dy[in]),
                &H->a(in, 0),
                &R->a(0,in),
                &(correctorStep->a(0,0)),
                &a,
                &(gainVector->a(0,0)),
                this
        );

        for(int ix = 0; ix < stateVector->w; ix++)
        {
            corrector->a(0,ix) += correctorStep->a(0,ix);
            if(iy < y->number - 1 )
            {
                int nextIndex = y->index[iy+1];
                y->dy[nextIndex] -= H->a(nextIndex, ix) * corrector->a(0,ix);
            }
        }
    }

    for(int ix = 0; ix < stateVector->w; ix++)
    {
        stateVectorInnovation->a(0,ix) = stateVector->a(0,ix) + corrector->a(0,ix);
    }

    delete corrector;
    delete correctorStep;
    delete gainVector;

}

/**
 *
 *  For liner Kalman model this function does the prediction of
 *  state with respect to control vector
 *
 *  \f[x_k = F_k * x_{k-1} + B_k * u_k \f]
 *
 *  For an extended Kalman Filter you should calculate
 *
 *  \f[x_k = f(x_{k-1}, u_k) \f]
 *
 *  instead
 *
 *
 *  Autor  : Schick           22.2.90
 *  Changes: Stein            20.1.94
 *  Changes: A Pimenov        17.2.10
 *
 *
 * \param F                      Transition Matrix
 * \param B                      Control-input model matrix
 * \param u                      Control vector
 * \param stateVector            State vector
 * \param stateVectorPredition   State vector prediction
 * \param stepsToMake            Number of cycles Forecast
 */
void KalmanFilter::modelStateTransition(
        COOSparseMatrix *F,
        COOSparseMatrix *B,
        Matrix    *u,
        Matrix    *stateVector,
        Matrix    *stateVectorPredition,
        int       stepsToMake
      )
{

    ASSERT_TRUE (F, "F matrix cannot be NULL");
    ASSERT_TRUE (stateVector, "State vector cannot be NULL");
    ASSERT_TRUE (stateVectorPredition, "State vector prediction matrix cannot be NULL");

    ASSERT_TRUE (stateVector->h == 1,          "state should be vector (h != 1)");
    ASSERT_TRUE (stateVectorPredition->h == 1, "state prediction should be vector (h != 1)");

    ASSERT_TRUE (stateVector->w == stateVectorPredition->w, "state and state prediction should have the same size");

    int i, j, ir, ic, t;

    for (t = 0; t < stepsToMake; t++) {
        /* TODO: Move two blocks below to separate function in the sparseMatrix class */
        stateVectorPredition->fillWith(0);
        j = (int)F->elements.size();

        for (i = 0; i < j; i++) {
            ir = F->elements[i].row;
            ic = F->elements[i].column;
            stateVectorPredition->a(0,ir) += F->elements[i].value * stateVector->a(0,ic);
        }

        /*TODO: Think about uniting B and u data structures*/
        if (B != NULL) {

            /* Bu */
            j = (int)B->elements.size();

            for (i = 0; i < j; i++) {
                ir = B->elements[i].row;
                ic = B->elements[i].column;
                stateVectorPredition->a(0,ir) += B->elements[i].value * u->a(0,ic);
            }
        }
    }
}

void KalmanFilter::step (Matrix *R, DyVector *y, Matrix *C)
{
    /* Now we have predicted filled and got new innovation data */
    calculatePrediction(P, Q, F);
    oldState->fillWith(currentState);
    modelStateTransition(F, NULL, NULL, oldState, predictedState);
    calculateInnovation(R, P, C, y, predictedState, currentState);
}

} //namespace corecvs

