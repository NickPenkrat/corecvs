#include <stdio.h>
#include <string>

#include <QtCore/QCoreApplication>
#include "QTimer"

#include "main_grab_N_captures.h"
#include "core/framesources/cameraControlParameters.h"
#include "core/reflection/commandLineSetter.h"

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


static void usage()
{
    printf("\n\n\n"
        " Info supported params:\n"
        "                 test_grab_N_captures --info v4l2:/dev/video1 v4l2:/dev/video0\n"
        "\n"
        " Capture frames: test_grab_N_captures v4l2:/dev/video1  v4l2:/dev/video0\n"
        " Capture frames: test_grab_N_captures 0 1\n"
        "\n"
        " To set additional params:\n"
        "\t\tExposure:      --exposure=VALUE\n"
        "\t\tWhite balance: --whiteBalance=VALUE\n"
        "\t\tGain:          --gain=VALUE\n"
        "\n"
        "\t\tVALUE - in range from min to max for param\n");
}

int main(int argc, char **argv)
{

    bool getParam = false;

    QCoreApplication app(argc, argv);

    CommandLineSetter s(argc, argv);
    bool help = s.getBool("help");
    if (help)
    {
        usage();
        return 0;
    }

    bool info = s.getBool("info");

    vector<std::string>                 deviceNames;

    int gain         = s.getInt("gain"        , -1);
    int exposure     = s.getInt("exposure"    , -1);
    int whiteBalance = s.getInt("whiteBalance", -1);

    int skip = s.getInt("skip", 0);



    printf("\n\n");
    printf("Read command-line\n");

    printf("EXPOSURE %i \n"     , exposure);
    printf("WHITE_BALANCE %i \n", whiteBalance);
    printf("GAIN %i \n"         , gain);
    printf("Skip %i \n"         , skip);

    deviceNames = s.nonPrefix();

    deviceNames.erase(deviceNames.begin());

    if (deviceNames.empty())
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

    Waiter *waiter = new Waiter;

    for (size_t i = 0; i < deviceNames.size(); i++)
    {
        std::string captureString = deviceNames[i];
        printf("Attempting a grab __________    %s    _________\n", captureString.c_str());

        Waiter::CameraDescriptor camDesc;
        camDesc.camId = (int)i;

        camDesc.input = ImageCaptureInterfaceQtFactory::fabric(captureString, true);

        camDesc.result = NULL;
        camDesc.toSkip = skip;

        ImageCaptureInterface::CapErrorCode result = ImageCaptureInterface::FAILURE;
        if (camDesc.input != NULL) {
            result = camDesc.input->initCapture();
        } else {
             SYNC_PRINT(("I can`t fabricate the camera %s\n", captureString.c_str()));
        }

        if (result == ImageCaptureInterface::FAILURE)
        {
            SYNC_PRINT(("I can`t init the camera %s\n", captureString.c_str()));
            return 0;
        }
        else
        {
            SYNC_PRINT(("Camera %s init success\n", captureString.c_str()));
        }
        waiter->mCaptureInterfaces.append(camDesc);
        QObject::connect(camDesc.input, SIGNAL(newFrameReady(frame_data_t)), waiter->mCaptureMapper, SLOT(map()), Qt::QueuedConnection);
        waiter->mCaptureMapper->setMapping(camDesc.input, waiter->mCaptureInterfaces.count() - 1);

        SYNC_PRINT(("Signals connected"));
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
