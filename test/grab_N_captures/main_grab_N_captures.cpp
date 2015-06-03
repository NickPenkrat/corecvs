#include "main_grab_N_captures.h"

int main (int argc, char **argv)
{
  printf("Attempting a grab 1 ____________________________________________\n");
  QCoreApplication app(argc, argv);

    vector<ImageCaptureInterface*> captures;
    vector<Waiter*> waiters;
    vector<QMutex*> mutexs;

    for(int i = 1; i < argc; i++)
    {
        char * captureString = argv[i];

        QMutex *mutex = new QMutex();
        ImageCaptureInterface *input = ImageCaptureInterface::fabric(captureString, 1);
        ImageCaptureInterface::CapErrorCode res = input->initCapture();

        printf("%s\n\n",captureString);

        if (ImageCaptureInterface::FAILURE == res)
        {
            printf("BaseHostDialog::initCapture(): Error: none of the capture devices started.\n");
            return 0;
        }
        else if (ImageCaptureInterface::SUCCESS_1CAM == res)
        {
            printf("BaseHostDialog::initCapture(): Will be using only one capture device.\n");
        }
        Waiter *waiter = new Waiter;
        waiter->input = input;
        waiter->mutex = mutex;
        QObject::connect(input, SIGNAL(newFrameReady(frame_data_t)), waiter, SLOT(onFrameReady()));
        input->startCapture();

        waiters.push_back(waiter);
        captures.push_back(input);
        mutexs.push_back(mutex);
    }

    QTimer::singleShot(2000, &app, SLOT(quit()));
    return app.exec();
}
