#include "main_grab_N_captures.h"
#include "QTimer"

int main (int argc, char **argv)
{
  printf("Attempting a grab 1 ____________________________________________\n");
  QCoreApplication app(argc, argv);

    vector<ImageCaptureInterface*> captures;
    vector<Waiter*> waiters;

    for(int i = 1; i < argc; i++)
    {

      char * captureString = argv[i];
      ImageCaptureInterface *input = ImageCaptureInterface::fabric(captureString, 1);//"dshow:1:rgb", 1);
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
      QObject::connect(input, SIGNAL(newFrameReady(frame_data_t)), waiter, SLOT(onFrameReady()));
      input->startCapture();

        waiters.push_back(waiter);
        captures.push_back(input);
    }

    QTimer::singleShot(5000, &app, SLOT(quit()));

  printf("+++++++++++++++++++++++++++++++++++");

  return app.exec();

//    ++++++++++++++++++++++++++++++++++
//    ++++++++++++++++++++++++++++++++++
//    ++++++++++++++++++++++++++++++++++
//    ++++++++++++++++++++++++++++++++++

//    printf("Attempting a grab 1\n");
//    QCoreApplication app(argc, argv);

//    ImageCaptureInterface *input = ImageCaptureInterface::fabric("dshow:1,2:1/10", 1);
//    ImageCaptureInterface::CapErrorCode res = input->initCapture();
//    if (ImageCaptureInterface::FAILURE == res)
//    {
//        printf("BaseHostDialog::initCapture(): Error: none of the capture devices started.\n");
//        return 0;
//    }
//    else if (ImageCaptureInterface::SUCCESS_1CAM == res)
//    {
//        printf("BaseHostDialog::initCapture(): Will be using only one capture device.\n");
//    }

//    Waiter waiter;
//    waiter.input = input;
//    QObject::connect(input, SIGNAL(newFrameReady(frame_data_t)), &waiter, SLOT(onFrameReady()));

//    input->startCapture();

//    printf("+++++++++++++++++++++++++++++++++++");

//    app.exec();
//    return 0;


}
