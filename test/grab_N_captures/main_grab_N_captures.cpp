#include "main_grab_N_captures.h"

int main (int argc, char **argv)
{
    printf("Attempting a grab 1\n");
    QCoreApplication app(argc, argv);

    ImageCaptureInterface *input = ImageCaptureInterface::fabric("dshow:1,2:1/10", 1);
    ImageCaptureInterface::CapErrorCode res = input->initCapture();
    if (ImageCaptureInterface::FAILURE == res)
    {
        printf("BaseHostDialog::initCapture(): Error: none of the capture devices started.\n");
        return 0;
    }
    else if (ImageCaptureInterface::SUCCESS_1CAM == res)
    {
        printf("BaseHostDialog::initCapture(): Will be using only one capture device.\n");
    }

    Waiter waiter;
    waiter.input = input;
    QObject::connect(input, SIGNAL(newFrameReady(frame_data_t)), &waiter, SLOT(onFrameReady()));

    input->startCapture();
    app.exec();
	return 0;


}
