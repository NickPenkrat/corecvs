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

/* Temporary solution. This need to be hidden inside image capture interface */
#ifdef Q_OS_WIN
#ifdef WITH_DIRECTSHOW
#include "directShowCapture.h"
#define CAPTURE_INTERFACE DirectShowCaptureInterface
#endif
#else
#include "V4L2Capture.h"
#define CAPTURE_INTERFACE V4L2CaptureInterface
#endif


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

static bool cmdIfOption(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

static void usage()
{
    printf("\n\n\n"
        " Info supported params:\n"
        "                 test_grab_N_captures --info v4l2:/dev/video1  v4l2:/dev/video0\n"
        "\n"
        " Capture frames: test_grab_N_captures v4l2:/dev/video1  v4l2:/dev/video0\n"
        " Capture frames: test_grab_N_captures 0 1\n"
        "\n"
        " To set additional params:\n"
        "\t\tExposure:      --exposure:VALUE\n"
        "\t\tWhite balance: --whiteBalance:VALUE\n"
        "\t\tGain:          --gain:VALUE\n"
        "\n"
        "\t\tVALUE - in range from min to max for param\n");
}

int main(int argc, char **argv)
{
    bool info = false;
    bool getParam = false;

    QCoreApplication app(argc, argv);

    if (cmdIfOption(argv, argv + argc, "--help"))
    {
        usage();
        return 0;
    }

    if (cmdIfOption(argv, argv + argc, "--info"))
    {
        info = true;
    }

    vector<cchar *>                 deviceNames;
    vector<ImageCaptureInterface *> captures;
    vector<int *>                   frameToSkipList;

    int gain = -1;                 // getIntCmdOption(argv, argv + argc, "--exposure:");
    int exposure = -1;
    int whiteBalance = -1;

    printf("\n\n");
    printf("Read command-line\n");

    if (argc < 2)
    {
        printf("There's no given camera to read. Using the default one.\n");
#ifdef WIN32
        cchar* deviceDef = "0";
#else
        cchar* deviceDef = "v4l2:/dev/video0";
#endif
        deviceNames.push_back(deviceDef);
        printf("Device %s added.\n", deviceDef);
    }

    for (int i = 1; i < argc; i++)
    {
        string str(argv[i]);

        printf("Read %s\n", str.c_str());

        if (getIntCmdOption(argv[i], "--exposure:", &exposure))
        {
            getParam = true;
            printf("EXPOSURE %i \n", exposure);
        }
        else if (getIntCmdOption(argv[i], "--whiteBalance:", &whiteBalance))
        {
            getParam = true;
            printf("WHITE_BALANCE %i \n", whiteBalance);
        }
        else if (getIntCmdOption(argv[i], "--gain:", &gain))
        {
            getParam = true;
            printf("GAIN %i \n", gain);
        }
        else
        {
            deviceNames.push_back(argv[i]);
            printf("Device %s added.\n", argv[i]);
        }
    }

    Waiter *waiter = new Waiter;

    for (size_t i = 0; i < deviceNames.size(); i++)
    {
        cchar * captureString = deviceNames[i];
        printf("Attempting a grab __________    %s    _________\n", captureString);

        Waiter::CameraDescriptor camDesc;
        camDesc.camId = i;

        camDesc.input = new CAPTURE_INTERFACE(captureString,
                            1944,
                            2592,
                            8,
                            true);  // rgb mode
        camDesc.result = NULL;
        camDesc.toSkip = 10;

        ImageCaptureInterface::CapErrorCode result = camDesc.input->initCapture();
        if (result != ImageCaptureInterface::SUCCESS_1CAM)
        {
            printf("I can`t open the camera %s", captureString);
            return 0;
        }
        else
        {
            printf("Camera %s init success", captureString);
        }
        waiter->mCaptureInterfaces.append(camDesc);
        QObject::connect(camDesc.input, SIGNAL(newFrameReady(frame_data_t)), waiter->mCaptureMapper, SLOT(map()));
        waiter->mCaptureMapper->setMapping(camDesc.input, waiter->mCaptureInterfaces.count() - 1);

        CameraParameters mCameraParameters;
        if (getParam) {
            camDesc.input->queryCameraParameters(mCameraParameters);
        }

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
            camDesc.input->setCaptureProperty(mCameraParameters.EXPOSURE_AUTO, 0);
            camDesc.input->setCaptureProperty(mCameraParameters.EXPOSURE, exposure);
            printf("EXPOSURE %i set.\n", exposure);
        }
        if (gain > -1)
        {
            camDesc.input->setCaptureProperty(mCameraParameters.GAIN_AUTO, 0);
            camDesc.input->setCaptureProperty(mCameraParameters.GAIN, gain);
            printf("GAIN %i set.\n", gain);
        }
        if (whiteBalance > -1)
        {
            camDesc.input->setCaptureProperty(mCameraParameters.AUTO_WHITE_BALANCE, 0);
            camDesc.input->setCaptureProperty(mCameraParameters.WHITE_BALANCE, whiteBalance);
            printf("WB %i set.\n", whiteBalance);
        }

    }

    if (!waiter->mCaptureInterfaces.empty())
    {
        qDebug() << "Starting first camera";
        waiter->mCurrentCam = 0;
        waiter->mCaptureInterfaces[0].input->startCapture();
    }
    else {
        waiter->finilizeCapture();
    }

    QTimer::singleShot(20000, &app, SLOT(quit()));
    return app.exec();
}
