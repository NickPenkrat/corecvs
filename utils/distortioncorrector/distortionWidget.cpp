#include <QTableWidgetItem>

#include "log.h"
#include "rgb24Buffer.h"
#include "curvatureFunc.h"
#include "radialFunc.h"
#include "qtHelper.h"

#include "camerasCalibration/camerasCalibrationFunc.h"

#include "g12Image.h"
#include "displacementBuffer.h"
#include "angleFunction.h"
#include "anglePointsFunction.h"
#include "distortionWidget.h"
#include "ui_distortionWidget.h"
#include "distPointsFunction.h"

#ifdef WITH_OPENCV
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/highgui/highgui.hpp"
# include <opencv2/calib3d/calib3d.hpp>
# include "opencv2/core/core_c.h"
# include "OpenCVTools.h"

using namespace cv;

#endif

using corecvs::DistPointsFunction;


const double EPSILON = 0.0005;

DistortionWidget::DistortionWidget(QWidget *parent) :
      QWidget(parent)
    , mBufferInput(NULL)
    , mBufferWithCorners(NULL)
    , mDistortionParameters(NULL)
    , mDistortionCorrectTransform(NULL)
    , mResult(NULL)
    , mUi(new Ui::DistortionWidget)
{
    mUi->setupUi(this);
    this->setWindowTitle("Distortion correction");

    connect(mUi->widget, SIGNAL(newPolygonPointSelected(int,QPointF)), this, SLOT(tryAddPointToPolygon(int,QPointF)));
    connect(mUi->widget, SIGNAL(newPointFSelected(int, QPointF)), this, SLOT(tryAddPoint(int, QPointF)));
    connect(mUi->widget, SIGNAL(existedPointSelected(int, QPointF)), this, SLOT(initExistingPoint(int,QPointF)));
    connect(mUi->widget, SIGNAL(editPoint(QPointF, QPointF)), this, SLOT(editPoint(QPointF,QPointF)));

  //  connect(mUi->pointsTableWidget, SIGNAL(cellDoubleClicked(int,int)), this, SLOT(choosePoint(int,int)));
  //  connect(mUi->saveButton,             SIGNAL(released()), this, SLOT(addVector()));
  //  connect(mUi->deleteButton,           SIGNAL(released()), this, SLOT(deletePointPair()));

    connect(mUi->recalculateButton,      SIGNAL(released()), this, SLOT(initTransform()));
    connect(mUi->resetButton,            SIGNAL(released()), this, SLOT(resetParameters()));
    connect(mUi->setParamsButton,        SIGNAL(released()), this, SLOT(setParams()));
    connect(mUi->cornerDetectorButton,   SIGNAL(released()), this, SLOT(detectCorners()));
    connect(mUi->crossDetectorButton,    SIGNAL(released()), this, SLOT(detectCheckerboard()));
    connect(mUi->updateScoreButton,      SIGNAL(released()), this, SLOT(updateScore()));
    mDistortionParameters = new DistortionParameters();

    CheckerboardDetectionParameters params;
    mUi->checkerboardParametersWidget->setParameters(params);
    mUi->calibrationFeatures->loadPoints();
}

void DistortionWidget::resetParameters()
{
    mUi->bufferSelectorBox->setCurrentIndex(0);
    clearParameters();
    mDistortionCorrectTransform = QSharedPointer<DisplacementBuffer>(
            new DisplacementBuffer(mBufferInput->h, mBufferInput->w)
    );
    emit recalculationFinished(mDistortionCorrectTransform);
}

DistortionWidget::~DistortionWidget()
{
    mUi->calibrationFeatures->savePoints();

    mDistortionCorrectTransform.clear();
    delete_safe(mResult);
    delete_safe(mDistortionParameters);
    delete_safe(mBufferWithCorners);
    delete_safe(mBufferInput);
    delete_safe(mUi);
}

void DistortionWidget::setParams()
{
    delete_safe(mDistortionParameters);
    mDistortionParameters = new DistortionParameters(
            mUi->radiusSpinBox->value(),
            mUi->dsbScale->value());
}

void DistortionWidget::initTransform()
{
    switch (mUi->methodSelectorBox->currentIndex())
    {
    case 0:
        doTransformLM();
        break;
    case 1:
        doInversionTransform();
        break;
    case 2:
        doLinesTransform();
        break;
    case 3:
        doDefaultTransform();
        break;
    case 4:
        doManualTransform();
        break;
    }
    mUi->bufferSelectorBox->setCurrentIndex(2);
    showBufferChanged();
    emit recalculationFinished(mDistortionCorrectTransform);
}

void DistortionWidget::clearParameters()
{
    mUi->widget->clear();
    mUi->calibrationFeatures->clearObservationPoints();

    delete mDistortionParameters;
    mDistortionParameters = new DistortionParameters();
}


/**
 *  Distortion reconstruction with 3D template
 **/

/**
 *  Using corner detector as a helper
 **/

void DistortionWidget::detectCorners()
{
    if(mBufferInput == NULL) {
        return;
    }

    G12Buffer *grayChannel = mBufferInput->toG12Buffer();
    SpatialGradient *grad = new SpatialGradient(grayChannel);

    delete_safe(mBufferWithCorners);
    G12Buffer *corners = grad->findCornerPoints(mUi->lambdaDivisorBox->value());
    mUi->bufferSelectorBox->setCurrentIndex(1);

    CornerSegmentator segmentator(mUi->cornerThresholdBox->value());
    mResult = segmentator.segment<G12Buffer>(corners);
    foreach (CornerSegment *cornerSegment, *mResult->segments)
    {
        addPointPair(Vector3dd(), cornerSegment->getMean());
    }

    mBufferWithCorners = new RGB24Buffer(corners);

    delete_safe(corners);
    delete_safe(grad);
    delete_safe(grayChannel);
}

void DistortionWidget::detectCheckerboard()
{
#ifdef WITH_OPENCV
    if(mBufferInput == NULL) {
        return;
    }

    G12Buffer *grayChannel = mBufferInput->toG12Buffer();

    IplImage *inputIpl = OpenCVTools::getCVImageFromG12Buffer(grayChannel);
    int             found;
    vector<Point2f> pointbuf;
    Mat             view = cv::Mat(inputIpl, false);

    CheckerboardDetectionParameters params;
    mUi->checkerboardParametersWidget->getParameters(params);

    int chessV = params.vCrossesCount();
    int chessH = params.hCrossesCount();
    int cellSize = params.cellSize();

    Size boardSize(chessV, chessH);

    found = findChessboardCorners( view, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH );

    if(found)
    {
        L_INFO_P("Cross detected: %i", found);

        delete mBufferWithCorners;
        drawChessboardCorners(view, boardSize, Mat(pointbuf), found);

        IplImage viewImage = view;
        mBufferWithCorners = OpenCVTools::getRGB24BufferFromCVImage(&viewImage);

        PaintImageWidget *canvas = mUi->widget;

        if(params.cleanExisting())
        {
            mUi->calibrationFeatures->clearObservationPoints();
            canvas->mFeatures.mPaths.clear();
        }

        for(int ih = 0; ih < chessH; ih++)
        {
/*            canvas->mPaths.append(PaintImageWidget::VertexPath());
            PaintImageWidget::VertexPath &path = canvas->mPaths.last();
            for(int iw = 0; iw < chessV; iw++)
            {
                Vector2dd point(pointbuf.at(ih * chessV + iw).x,pointbuf.at(ih * chessV + iw).y);
                addPointPair(Vector3dd(cellSize * ih, cellSize * iw, 0), point);
                canvas->addVertex(point);
                canvas->addVertexToPath(&canvas->mPoints.last(), &path);
            } */
        }

        for(int iw = 0; iw < chessV; iw++)
        {
/*            canvas->mPaths.append(PaintImageWidget::VertexPath());
            PaintImageWidget::VertexPath &path = canvas->mPaths.last();

            for(int ih = 0; ih < chessH; ih++)
            {
                Vector2dd point(pointbuf.at(ih * chessV + iw).x,pointbuf.at(ih * chessV + iw).y);
                addPointPair(Vector3dd(cellSize * ih, cellSize * iw, 0), point);
                canvas->addVertex(point);
                canvas->addVertexToPath(&canvas->mPoints.last(), &path);
            }*/
        }
    }
    cvReleaseImage(&inputIpl);
    delete_safe(grayChannel);
    return;
#else
    return;
#endif
}

void DistortionWidget::tryAddPoint(int toolID, const QPointF &point)
{
    if (toolID != 0)
        return;
    mUi->calibrationFeatures->manualAddPoint(Qt2Core::Vector2ddFromQPointF(point));
}

void DistortionWidget::initExistingPoint(int toolID, const QPointF &point)
{
    if (toolID != 0) {
        return;
    }

    /*!!!!!!!*/
    /*
    for (int i = 0; i < mUi->pointsTableWidget->rowCount(); i ++)
    {
        QPointF tablePoint(mUi->pointsTableWidget->item(i, 3)->text().toDouble(), mUi->pointsTableWidget->item(i, 4)->text().toDouble());
        if ((point - tablePoint).manhattanLength() < EPSILON)
        {
            choosePoint(i, 0);
            return;
        }
    }
    */
}

void DistortionWidget::setBuffer(G12Buffer *buffer)
{
    if (buffer == NULL) {
        return;
    }
    delete_safe(mBufferInput);
    mBufferInput = new RGB24Buffer(buffer);
    showBufferChanged();
}

void DistortionWidget::setBuffer(RGB24Buffer *buffer)
{
    if (buffer == NULL) {
        return;
    }
    delete_safe(mBufferInput);
    mBufferInput = new RGB24Buffer(buffer);
    showBufferChanged();
}




void DistortionWidget::doTransformLM()
{
    Vector2d32 center(mUi->widget->mImage.data()->width() * 0.5, mUi->widget->mImage.data()->height() * 0.5);
    vector<double> values;
    vector<Vector3dd> arguments;
    foreach (PointObservation observation, mUi->calibrationFeatures->observationList)
    {
        values.push_back(observation.u());
        values.push_back(observation.v());
        arguments.push_back(observation.point);
    }
    CamerasCalibrationFunc func(&arguments, center);
    vector<double> bestParams;

    double params[] = {0, 0, 0, 0, -5.09, 11.1, -30.4, 35, 0.12, 0.04, -4.81, 14.6, -11.2, 11.2};
    vector<double> parameters(params, params + CORE_COUNT_OF(params));
    LevenbergMarquardt LMTransform(10000, 4, 1.2);
    LMTransform.f = &func;
    func.setIsDistored(false);
    bestParams = LMTransform.fit(parameters,values);
    func.setIsDistored(true);
    bestParams = LMTransform.fit(bestParams, values);
    vector<double> polynomKoeff;
    polynomKoeff.push_back(bestParams.at(0));
    polynomKoeff.push_back(bestParams.at(1));
//    LensDistortionModelParameters lenCorrectParams(polynomKoeff, bestParams.at(2), bestParams.at(3), center);

    LensDistortionModelParameters lenCorrectParams(
                center.x(),
                center.y(),
                bestParams.at(2), bestParams.at(3),
                polynomKoeff,
                1.0,
                1.0);


    RadialCorrection correction(lenCorrectParams);
    mDistortionCorrectTransform = QSharedPointer<DisplacementBuffer>(
            new DisplacementBuffer(&correction, mBufferInput->h, mBufferInput->w, false)
    );
}

/**
 *   Correction by Maria Osechkina's algorithm
 **/

void DistortionWidget::doInversionTransform()
{
    Vector2dd centre(mUi->widget->mImage.data()->width() * 0.5, mUi->widget->mImage.data()->height() * 0.5);
    vector<Vector2dd> points = mDistortionParameters->getPoints();

    DistortionCorrectTransform *transform = NULL;

    if (points.size() < 3 || !mDistortionParameters->needCalculateParams())
    {
        transform = new DistortionCorrectTransform(
            centre,
            mDistortionParameters->getRadius(),
            mDistortionParameters->getScale());
    }
    else
    {
        transform = new DistortionCorrectTransform(centre);
        transform->setRadius(points.at(0), points.at(1), points.at(2));
    }

    mDistortionCorrectTransform = QSharedPointer<DisplacementBuffer>(
            new DisplacementBuffer(transform, mBufferInput->h, mBufferInput->w)
    );
    delete_safe(transform);
}

/**
 *   Correction by algorithm of straight
 **/
void DistortionWidget::doLinesTransform()
{
    LineDistortionEstimatorParameters params;
    mUi->lineDistortionWidget->getParameters(params);


    PaintImageWidget *editor = mUi->widget;
    vector<vector<Vector2dd> > straights = editor->getPaths();
    L_INFO_P("Starting distortion calibration on %d lines", straights.size());
/*
    RadialCorrection correction(LensDistortionModelParameters(
       vector<double>(mUi->degreeSpinBox->value()),
       0.0, 0.0,
       1.0,
       center.l2Metric(),
       center
    ));
 */

    Vector2dd center(mBufferInput->w / 2.0, mBufferInput->h /2.0);

    RadialCorrection correction(LensDistortionModelParameters(
       center.x(),
       center.y(),
       0.0 ,0.0,
       vector<double>(params.polinomDegree()),
       1.0,
       1.0
    ));


    ModelToRadialCorrection modelFactory(
        correction,
        params.estimateCenter(),
        params.estimateTangent(),
        params.polinomDegree()
    );

    FunctionArgs *costFuntion = NULL;
    if (params.costAlgorithm() == LineDistortionEstimatorCost::JOINT_ANGLE_COST) {
        costFuntion = new AnglePointsFunction (straights, modelFactory);
    } else {
        costFuntion = new DistPointsFunction  (straights, modelFactory);
    }

    LevenbergMarquardt straightLevMarq(params.iterationNumber(), 2, 1.5);
    straightLevMarq.f = costFuntion;

    /* First aproximation is zero vector */
    vector<double> first(costFuntion->inputs, 0);
    modelFactory.getModel(correction, &first[0]);

    vector<double> value(costFuntion->outputs, 0);
    vector<double> straightParams = straightLevMarq.fit(first, value);

    RadialCorrection mLinesRadialCoorection = modelFactory.getRadial(&straightParams[0]);

    L_INFO_P("Inverting the distortion buffer");

   mDistortionCorrectTransform = QSharedPointer<DisplacementBuffer>(DisplacementBuffer::CacheInverse(&mLinesRadialCoorection,
            mBufferInput->h, mBufferInput->w,
            0.0,0.0,
            (double)mBufferInput->w, (double)mBufferInput->h,
            0.5, false)
   );

    mUi->isInverseCheckBox->setChecked(true);

    mUi->lensCorrectionWidget->setParameters(mLinesRadialCoorection.mParams);
    L_INFO_P("Ending distortion calibration");
    updateScore();
}

void DistortionWidget::updateScore()
{
    PaintImageWidget *editor = mUi->widget;
    vector<vector<Vector2dd> > straights = editor->getPaths();

    RadialCorrection radialCorrection;
    mUi->lensCorrectionWidget->getParameters(radialCorrection.mParams);

    LineDistortionEstimatorParameters params;
    mUi->lineDistortionWidget->getParameters(params);

    ModelToRadialCorrection modelFactory(
        radialCorrection,
        params.estimateCenter(),
        params.estimateTangent(),
        params.polinomDegree()
    );

    FunctionArgs *costFuntion = NULL;
    if (params.costAlgorithm() == LineDistortionEstimatorCost::JOINT_ANGLE_COST) {
        costFuntion = new AnglePointsFunction (straights, modelFactory);
    } else {
        costFuntion = new DistPointsFunction  (straights, modelFactory);
    }

    vector<double> modelParameters(costFuntion->inputs, 0);
    modelFactory.getModel(radialCorrection, &modelParameters[0]);

    vector<double> result(costFuntion->outputs);
    costFuntion->operator()(&modelParameters[0], &result[0]);

    double sqSum = 0;
    double maxValue = 0.0;
    for (unsigned i = 0; i < result.size(); i++) {
        if (fabs(result[i]) > maxValue) {
            maxValue = fabs(result[i]);
        }
        sqSum += result[i] * result[i];
    }

    int count = 0;
    PaintImageWidget *canvas = mUi->widget;
    for (unsigned i = 0; i < (unsigned)canvas->mFeatures.mPaths.size(); i++)
    {
        SelectableGeometryFeatures::VertexPath *path = canvas->mFeatures.mPaths[i];
        if (path->vertexes.size() < 3) {
            continue;
        }
        if (params.costAlgorithm() == LineDistortionEstimatorCost::JOINT_ANGLE_COST)
        {
            for (unsigned j = 1; j < path->vertexes.size() - 1; j++) {
                path->vertexes[j]->weight = fabs(result[count]) / maxValue;
                count++;
            }
        } else {
            for (unsigned j = 0; j < path->vertexes.size(); j++) {
                path->vertexes[j]->weight = fabs(result[count]) / maxValue;
                count++;
            }
        }
    }
    canvas->update();

    sqSum /= result.size();

    mUi->scoreLabel->setText(QString::number(sqrt(sqSum)));
}

QSharedPointer<DisplacementBuffer> DistortionWidget::distortionCorrectionTransform()
{
    return mDistortionCorrectTransform;
}

void DistortionWidget::showBufferChanged()
{
    int id = mUi->bufferSelectorBox->currentIndex();
    RGB24Image *image = NULL;
    switch (id)
    {
    case 0:
        if (mBufferInput != NULL)
        {
            image = new RGB24Image(mBufferInput);
            mUi->widget->setImage(QSharedPointer<QImage>(image));
        }
        break;
    case 1:
        if (mBufferWithCorners != NULL)
        {
            image = new RGB24Image(mBufferWithCorners);
            mUi->widget->setImage(QSharedPointer<QImage>(image));
        }
        break;
    case 2:
        {
            if (mDistortionCorrectTransform == NULL) {
                break;
            }
            RGB24Buffer *buffer = mBufferInput->doReverseDeformationBl<RGB24Buffer, DisplacementBuffer>(
                        mDistortionCorrectTransform.data(), mBufferInput->h, mBufferInput->w);
            image = new RGB24Image(buffer);
            QPainter painter(image);
            vector<vector<Vector2dd> > straights = mUi->widget->getPaths();

            RadialCorrection linesRadialCoorection;
            mUi->lensCorrectionWidget->getParameters(linesRadialCoorection.mParams);

            for (unsigned i = 0; i < straights.size(); i++) {
                for (unsigned j = 0; j < straights[i].size(); j++) {
                    Vector2dd point = straights[i][j];
                    Vector2dd moved = linesRadialCoorection.map(point.y(), point.x());
                    painter.setPen(Qt::yellow);
                    painter.drawLine(moved.x() - 1, moved.y() - 1, moved.x() + 1, moved.y() + 1);
                    painter.drawLine(moved.x() + 1, moved.y() - 1, moved.x() - 1, moved.y() + 1);
//                    cout << point << "=>" << moved << "\n";
                }
            }
            painter.end();

            mUi->widget->setImage(QSharedPointer<QImage>(image));
            delete_safe(buffer);
            break;
        }
    }
}

/*Seems like a trash*/
#if 0
void DistortionWidget::printVectorPair(const Vector3dd &key, const Vector2dd &value)
{
    QTableWidget *table = mUi->calibrationFeatures;
    for (int i = 0; i < table->rowCount(); i ++)
    {
        if (table->item(i, 3)->text().toDouble() == value.x() &&
            table->item(i, 4)->text().toDouble() == value.y())
        {
            table->item(i, 0)->setText(QString::number(key.x()));
            table->item(i, 1)->setText(QString::number(key.y()));
            table->item(i, 2)->setText(QString::number(key.z()));
            return;
        }
    }
    for (int i = 0; i < table->rowCount(); i ++)
    {
        for (int j = 0; j < table->columnCount(); j ++)
            table->item(i, j)->setBackgroundColor(Qt::color0);
    }
}
#endif

void DistortionWidget::addPointPair(const Vector3dd &key, const Vector2dd &value)
{
    PointObservation observation(key, value);
    mUi->calibrationFeatures->addObservation(observation);
    /*TODO!!!!*/
    //mUi->widget->addVertex(value);
}


void DistortionWidget::doDefaultTransform()
{
    const double scale = 1.05;
    const Vector2dd  center(mBufferInput->w * 0.5, mBufferInput->h * 0.5);
    const double k = 1024.0 / mBufferInput->h; //params for ueyecameras
    const double params[] = {
            0.000657414 * k,
           -6.97987e-05 * k * k,
             7.9655e-07 * pow(k, 3),
             -4.318e-09 * pow(k, 4),
            1.33988e-11 * pow(k, 5),
           -2.50562e-14 * pow(k, 6),
            2.79465e-17 * pow(k, 7),
           -1.71374e-20 * pow(k, 8),
             4.4494e-24 * pow(k, 9)};
    vector<double> straightParams(params, params + CORE_COUNT_OF(params));
    LensDistortionModelParameters lenCorrectionParams(
                center.x(),
                center.y(),
                0.0, 0.0,
                straightParams,
                1.0,
                scale
             );

    RadialCorrection radCorrection(lenCorrectionParams);
    mDistortionCorrectTransform = QSharedPointer<DisplacementBuffer>(
            new DisplacementBuffer(&radCorrection, mBufferInput->h, mBufferInput->w, true)
    );
}

/* Manual transform */

void DistortionWidget::doManualTransform()
{
    RadialCorrection linesRadialCoorection;
    mUi->lensCorrectionWidget->getParameters(linesRadialCoorection.mParams);

    if (!mUi->isInverseCheckBox->isChecked()) {
        mDistortionCorrectTransform = QSharedPointer<DisplacementBuffer>(new DisplacementBuffer(&linesRadialCoorection, mBufferInput->h, mBufferInput->w, true));
    } else {
        mDistortionCorrectTransform = QSharedPointer<DisplacementBuffer>(DisplacementBuffer::CacheInverse(&linesRadialCoorection,
            mBufferInput->h, mBufferInput->w,
            0.0,0.0,
            (double)mBufferInput->w, (double)mBufferInput->h, 0.5)
        );
    }
    updateScore();
}


