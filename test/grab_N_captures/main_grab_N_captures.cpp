#include <stdio.h>
#include <string>
//#ifdef WIN32
//# include <windows.h>
//# define tSleep  Sleep
//#else
//# include <unistd.h>
//# include <stdlib.h>
//# define tSleep _sleep
//#endif
#include <QtCore/QCoreApplication>
#include "QTimer"

#include "main_grab_N_captures.h"
#include "cameraControlParameters.h"


static bool getIntCmdOption(const std::string & value, const std::string & option, int *param)
{
    size_t  position = value.find(option);
    if (position != std::string::npos)
    {
        std::string strSub = value.substr (position + option.length());
        *param = atoi(strSub.c_str());
        return 1;
    }
    return 0;
}

static bool cmdIfHelp(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

int main(int argc, char **argv)
{
    bool info = false;
    QCoreApplication app(argc, argv);

    if (cmdIfHelp(argv, argv + argc, "--help"))
    {
        printf("\n\n\n Info supported params: test_grab_N_captures --info v4l2:/dev/video1  v4l2:/dev/video0\n");
        printf("\n Capture frames: test_grab_N_captures v4l2:/dev/video1  v4l2:/dev/video0\n");
        printf("\n Capture frames: test_grab_N_captures dshow:0,1\n");
        printf("\n\n To set additional params:\n");
        printf("\t\tExposure:      --exposure:VALUE\n");
        printf("\t\tWhite balance: --whiteBalance:VALUE\n");
        printf("\t\tGain:          --gain:VALUE\n");
        printf("\n\t\tVALUE - in range from min to max for param\n");
        return 0;
    }

    if (cmdIfHelp(argv, argv + argc, "--info"))
    {
        info = true;
    }

    vector<char*> deviceNames;
    vector<ImageCaptureInterface*> captures;
    vector<int*> frameToSkipList;

    int gain = -1;                 // getIntCmdOption(argv, argv + argc, "--exposure:");
    int exposure = -1;
    int whiteBalance = -1;

    printf("\n\n");
    printf("Read command-line\n");

    for (int i = 1; i < argc; i++)
    {
        string str(argv[i]);

        printf("Read %s\n", str.c_str());

        if (getIntCmdOption(argv[i], "--exposure:", &exposure))
        {
            printf("EXPOSURE %i \n", exposure);
        }
        else if (getIntCmdOption(argv[i], "--whiteBalance:", &whiteBalance))
        {
            printf("WHITE_BALANCE %i \n", whiteBalance);
        }
        else if (getIntCmdOption(argv[i], "--gain:", &gain))
        {
            printf("GAIN %i \n", gain);
        }
        else
        {
            deviceNames.push_back(argv[i]);
            printf("Device %s added.\n", argv[i]);
        }
    }

    for (int i = 0; i < deviceNames.size(); i++)
    {
        char * captureString = deviceNames[i];
        printf("Attempting a grab 1 __________    %s    _________\n", captureString);

        int frameToSkip = 8;
        ImageCaptureInterface *input = ImageCaptureInterface::fabric(captureString, 1);
        ImageCaptureInterface::CapErrorCode res = input->initCapture();

        if (ImageCaptureInterface::FAILURE == res)
        {
            printf("BaseHostDialog::initCapture(): Error: none of the capture devices started.\n");
            return 0;
        }
        if (ImageCaptureInterface::SUCCESS_1CAM == res)
        {
            printf("BaseHostDialog::initCapture(): Will be using only one capture device.\n");
        }

        Waiter *waiter = new Waiter;
        waiter->input = input;
        waiter->frameToSkip = &frameToSkip;

        QObject::connect(input, SIGNAL(newFrameReady(frame_data_t)), waiter, SLOT(onFrameReady()));

        CameraParameters mCameraParameters;
        input->queryCameraParameters(mCameraParameters);

        if (info)
        {
            // GET CAMERA PARAMETERS

            for (int i = CameraParameters::FIRST; i < CameraParameters::LAST; i++)
            {
                CaptureParameter &params = mCameraParameters.mCameraControls[i];
                if (!params.active()) {
                    continue;
                }
                string name(CameraParameters::names[i]);
                printf("P: %20s ---\t active: %i\t def:%4i in [%4i, %4i]\n",
                       name.c_str(),
                       (int)params.active(),
                       params.defaultValue(),
                       params.minimum(),
                       params.maximum());
            }
            return 0;
        }

        // SET CAMERA PARAMETERS

        if (exposure > -1)
        {
            input->setCaptureProperty(mCameraParameters.EXPOSURE_AUTO, 0);
            input->setCaptureProperty(mCameraParameters.EXPOSURE, exposure);
            printf("EXPOSURE %i set.\n", exposure);
        }
        if (gain > -1)
        {
            input->setCaptureProperty(mCameraParameters.GAIN_AUTO, 0);
            input->setCaptureProperty(mCameraParameters.GAIN, gain);
            printf("GAIN %i set.\n", gain);
        }
        if (whiteBalance > -1)
        {
            input->setCaptureProperty(mCameraParameters.AUTO_WHITE_BALANCE, 0);
            input->setCaptureProperty(mCameraParameters.WHITE_BALANCE, whiteBalance);
            printf("WB %i set.\n", whiteBalance);
        }

        input->startCapture();

        captures.push_back(input);
        frameToSkipList.push_back(&frameToSkip);
    }

    QTimer::singleShot(10000, &app, SLOT(quit()));
    return app.exec();
}
