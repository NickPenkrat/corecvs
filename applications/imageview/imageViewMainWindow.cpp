#include "imageViewMainWindow.h"
#include "ui_imageViewMainWindow.h"

ImageViewMainWindow::ImageViewMainWindow(QWidget *parent) :
    QWidget(parent),
    input(NULL),
    ui(new Ui::ImageViewMainWindow)
{
    ui->setupUi(this);
}

ImageViewMainWindow::~ImageViewMainWindow()
{
    delete ui;
}

void ImageViewMainWindow::paramsChanged()
{
    BitSelectorParameters params;
    ui->bitSelector->getParameters(params);






}
