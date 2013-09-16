#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

/* ===========================================================================
* MAGNA Electronics - C O N F I D E N T I A L
*  This document in its entirety is CONFIDENTIAL and may not be disclosed,
*  disseminated or distributed to parties outside MAGNA Electronics
*  without written permission from MAGNA Electronics.
* ===========================================================================
* SHORT:   Klm
* DESIGN:  <tbd>
* DESCRIPTION:
*    Kalman Filter
* ======================================================================== */

#include "global.h"

#include "cooSparseMatrix.h"
#include "upperUnitaryMatrix.h"
#include "uduDecomposed.h"

namespace corecvs {

class DyVector
{
public:
    explicit DyVector(int ymax);
    ~DyVector();


    int *index;          /*[YMAX]*/
    double *dy;          /*[YMAX]*/
    int number;          /*number*/
    int maxNumber;       /*Max number*/
};

class KalmanFilter
{
public:
    KalmanFilter(int stateSize, double stateNoise = 1.0);
    ~KalmanFilter();

    void calculatePrediction(
            UDUTDecomposed *P,
            Matrix    * Q,
            COOSparseMatrix *F);


    void calculateInnovation(
            Matrix *R,
            UDUTDecomposed *P,
            Matrix *measurementMatrix,
            DyVector* y,
            Matrix    *stateVector,
            Matrix    *stateVectorInnovation);


    void modelStateTransition(
            COOSparseMatrix *F,
            COOSparseMatrix *B,
            Matrix    *u,
            Matrix    *stateVector,
            Matrix    *stateVectorPredition,
            int        stepsToMake  = 1);


    void step (Matrix *R, DyVector *y, Matrix *C);

    COOSparseMatrix *F; /**< State evolution matrix */
    Matrix *Q;          /**< State noise */
    UDUTDecomposed *P;  /**< State covariance matrix*/

    Matrix *oldState;        /**< State for the previous cycle */
    Matrix *predictedState;  /**< Predicted state */
    Matrix *currentState;    /**< Current state after correction */



   /*=====================================================================================*/

    double *fv0; /* NMAX*NMAX == NPHIMAX */
    /* ?? Kalman-Filter Gain Vektor (вектор усиления Фильтра Калмана )Kalman filter gain vector*/
    double *fv1; /* NPHIMAX */
    /* ?? Korrektur der Zustandsgroessen (Коррекция переменных состояния) Correction of the state variables*/
    double *fv2; /* 2*NMAX */
    double *fv3; /* NMAX */
    int *iv0; /* NPHIMAX */
    int *iv1; /* NPHIMAX */

    int nmax;
    int nphimax; /* nmax * nmax */




private:
};

} //namespace corecvs

#endif /*_KALMAN_FILTER_H_*/





