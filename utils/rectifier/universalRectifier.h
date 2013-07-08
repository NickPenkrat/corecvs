#ifndef UNIVERSALRECTIFIER_H_
#define UNIVERSALRECTIFIER_H_
/**
 * \file universalRectifier.h
 * \brief Add Comment Here
 *
 * \date Oct 28, 2011
 * \author alexander
 */

#include <QtCore/QString>
#include <QtGui/QImage>

#include "rectifyParameters.h"
#include "matrix33.h"
#include "rectifyParameters.h"
#include "correspondanceList.h"
#include "cameraParameters.h"
#include "essentialMatrix.h"
#include "triangulator.h"


using namespace core3vi;

class UniversalRectifier
{
public:
    RectifyParameters params;
    /**
     *   The input list.
     *   Right to Left correspondances.
     *   For rectified pair all end.x() - start.x() should be positive
     *   After recalculation it will be altered with new flags
     *
     **/
    CorrespondanceList *inputList;
    /**
     *  This is a local copy of the input that we will alter with KLT, or any other
     *  improvement algorithm
     **/
    DerivedCorrespondanceList *list;

    /**
     *  This is a local copy of the input that we will alter with KLT, or any other
     *  improvement algorithm
     **/
    DerivedCorrespondanceList *postKlt;

    /**
     *  This is a local copy of the input that we will hold the filtered by length values
     **/
    DerivedCorrespondanceList *filteredList;


    /**
     *  This is a local copy of the input that we will hold the filtered by length values
     **/
    DerivedCorrespondanceList *normalList;

    /**
     *  Data in the format the estimators understand
     *
     **/
    vector<Correspondance *> *data;

    /**
     *  This is used for KLT only and should be brought out of the class
     *  TODO: Move this out
     *
     **/
    G12Buffer *left;
    G12Buffer *right;


    /**
     *  Z direction for rectification
     **/
    Vector3dd direction;

    RectificationResult result;

    /**************************************************************************
     *  Some helpful results
     **/
    /* Epipoles */
    Vector3dd E1;
    Vector3dd E2;

    QImage *zGraph;
    QImage *topView;

    /**************************************************************************
     * Error list and status
     **/
    QString status;
    int code;

    UniversalRectifier(RectifyParameters *_params):
        params(*_params),
        inputList(NULL),
        list(NULL),
        postKlt(NULL),
        filteredList(NULL),
        normalList(NULL),
        data(NULL),
        left(NULL),
        right(NULL),
        zGraph(NULL),
        topView(NULL),
        code(0)
    {};

    void recalculate(void);

    virtual ~UniversalRectifier();
private:

    void filter(void);
    void ransacMethod(double scale);
    void iterativeMethod(void);
    void manualMethod( void );
    void getZ(void);
    void decomposeEssential(void);

};

#endif /* UNIVERSALRECTIFIER_H_ */
