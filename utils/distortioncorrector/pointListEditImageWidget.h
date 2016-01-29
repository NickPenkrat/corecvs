#ifndef POINT_LIST_EDIT_IMAGE_WIDGET_H
#define POINT_LIST_EDIT_IMAGE_WIDGET_H

#include <QObject>

#include "advancedImageWidget.h"
#include "observationListModel.h"



class PointListEditImageWidget : public AdvancedImageWidget
{
   Q_OBJECT

public:
    enum PaintToolClass
    {
        MOVE_POINT_TOOL = TOOL_CLASS_LAST,
        ADD_POINT_TOOL,
        DEL_POINT_TOOL,
        PAINT_TOOL_CLASS_LAST
    };


   ObservationListModel *mObservationListModel;

   QToolButton *mAddButton;
   QToolButton *mMoveButton;
   QToolButton *mDeleteButton;
   QToolButton *mAddInfoButton;


   PointListEditImageWidget(QWidget *parent = NULL, bool showHeader = true);
   void setObservationModel(ObservationListModel *observationListModel);

   int mSelectedPoint;

   // AdvancedImageWidget interface
public slots:
   virtual void childRepaint(QPaintEvent *event, QWidget *who) override;
   virtual void toolButtonReleased(QWidget *button) override;
   virtual void childMousePressed(QMouseEvent *event) override;
   virtual void childMouseMoved(QMouseEvent *event) override;
   void invalidateModel();

protected:
   int findClosest(Vector2dd imagePoint, double limitDistance = numeric_limits<double>::max());
   Vector2dd getPointById(int row);
   QString   getMetaById (int row);
};

/**
 *
 **/
class PointListEditImageWidgetUnited : public AdvancedImageWidget
{
   Q_OBJECT

public:
    enum PaintToolClass
    {
        MOVE_POINT_TOOL = TOOL_CLASS_LAST,
        ADD_POINT_TOOL,
        DEL_POINT_TOOL,
        PAINT_TOOL_CLASS_LAST
    };


   PointImageEditorInterface *mObservationListModel;

   QToolButton *mAddButton;
   QToolButton *mMoveButton;
   QToolButton *mDeleteButton;
   QToolButton *mAddInfoButton;


   PointListEditImageWidgetUnited(QWidget *parent = NULL, bool showHeader = true);
   void setObservationModel(PointImageEditorInterface *observationListModel);

   int mSelectedPoint;

   // AdvancedImageWidget interface
public slots:
   virtual void childRepaint(QPaintEvent *event, QWidget *who) override;
   virtual void toolButtonReleased(QWidget *button) override;
   virtual void childMousePressed(QMouseEvent *event) override;
   virtual void childMouseMoved(QMouseEvent *event) override;
   void invalidateModel();

protected:
   int findClosest(Vector2dd imagePoint, double limitDistance = numeric_limits<double>::max());
};


#endif // POINT_LIST_EDIT_IMAGE_WIDGET_H