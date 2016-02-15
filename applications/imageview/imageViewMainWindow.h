#ifndef IMAGEVIEWMAINWINDOW_H
#define IMAGEVIEWMAINWINDOW_H

#include "rgbTBuffer.h"
#include "g12Buffer.h"

#include <QWidget>

namespace Ui {
class ImageViewMainWindow;
}

class ImageViewMainWindow : public QWidget
{
    Q_OBJECT

public:
    RGB48Buffer *input;
    explicit ImageViewMainWindow(QWidget *parent = 0);
    ~ImageViewMainWindow();

public slots:

    void paramsChanged(void);

private:
    Ui::ImageViewMainWindow *ui;
};

#endif // IMAGEVIEWMAINWINDOW_H
