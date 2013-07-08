#ifndef KALMAN_FILTER_BU_H
#define KALMAN_FILTER_BU_H

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

#include "cooSparseMatrix.h"
#include "upperUnitaryMatrix.h"
#include "uduDecomposed.h"

namespace core3vi {


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

class KalmanWorkspace
{
public:
    explicit KalmanWorkspace(int nmax);
    ~KalmanWorkspace();

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



    void calculatePrediction(
                         UDUTDecomposed *P,
                         Matrix    * Q,
                         COOSparseMatrix *F);


    void calculateInnovation(
                          Matrix *R,
                          UDUTDecomposed *P,
                          Matrix *measurementMatrix,
                          DyVector* b_y,
                          Matrix    *stateVector,
                          Matrix    *stateVectorInnovation);


    void modelStateTransition(COOSparseMatrix *transitionMatrix,
                             COOSparseMatrix *controlMatrix,
                             Matrix    *u,
                             Matrix    *stateVector,
                             Matrix    *stateVectorPredition,
                             int       stepsToMake  = 1);

private:
};

} //namespace core3vi
#endif /* KALMAN_FILTER_BU_H */



