#include "main_grab_N_captures.h"
#include "cameraControlParameters.h"
#include <string>

int info = 0;

bool getIntCmdOption(const std::string & value, const std::string & option, int *param)
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

bool cmdIfHelp(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}


int main (int argc, char **argv)
{
    QCoreApplication app(argc, argv);

    if(cmdIfHelp(argv, argv+argc, "--help")){
        printf("\n\n\n Info supported params: ./test_grab_N_captures --info v4l2:/dev/video1  v4l2:/dev/video0 \n");
        printf("\n Capture frames: ./test_grab_N_captures v4l2:/dev/video1  v4l2:/dev/video0 \n");
        printf("\n\n Set additional params: ./test_grab_N_captures v4l2:/dev/video1  v4l2:/dev/video0 \n");
        printf("           Exposure:      --exposure:VALUE\n");
        printf("           White balance: --whiteBalance:VALUE\n");
        printf("           Gain:          --gain:VALUE\n");
        printf("\n\n      VALUE - in range from min to max for param\n");
        return 0;
    }

    if(cmdIfHelp(argv, argv+argc, "--info")){
        info = 1;
    }

    vector<char*> deviceNames;
    vector<ImageCaptureInterface*> captures;
    vector<int*> frameToSkipList;

    int* GAIN = new int(-1); // getIntCmdOption(argv, argv + argc, "--exposure:");
    int* EXPOSURE = new int(-1);
    int* WHITE_BALANCE = new int(-1);

    printf("\n\n");
    printf("Read command-line\n");

    for(int i = 1; i < argc; i++)
    {
        string str(argv[i]);

        printf("Read %s\n", str.c_str());

        if(getIntCmdOption(argv[i], "--exposure:", EXPOSURE))
        {
            printf("EXPOSURE %i \n", *EXPOSURE);
        }
        else if(getIntCmdOption(argv[i], "--whiteBalance:", WHITE_BALANCE))
        {
            printf("WHITE_BALANCE %i \n", *WHITE_BALANCE);
        }
        else if(getIntCmdOption(argv[i], "--gain:", GAIN))
        {
            printf("GAIN %i \n", *GAIN);
        }
        else
        {
            deviceNames.push_back(argv[i]);
            printf("Device %s added.\n", argv[i]);
        }
    }

    for(int i = 0; i < deviceNames.size(); i++)
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
        else if (ImageCaptureInterface::SUCCESS_1CAM == res)
        {
            printf("BaseHostDialog::initCapture(): Will be using only one capture device.\n");
        }
        Waiter *waiter = new Waiter;
        waiter->input = input;
        waiter->frameToSkip = &frameToSkip;

        QObject::connect(input, SIGNAL(newFrameReady(frame_data_t)), waiter, SLOT(onFrameReady()));

        CameraParameters mCameraParameters;
        input->queryCameraParameters(mCameraParameters);

        if(info){
        ///////////////////////////////////////////////////////////
        /// GET CAMERA PARAMETERS
        ///////////////////////////////////////////////////////////

            for (int i = CameraParameters::FIRST; i < CameraParameters::LAST; i++)
            {
                CaptureParameter &params = mCameraParameters.mCameraControls[i];

                if (!params.active()) {
                    continue;
                }

                string name(CameraParameters::names[i]);

                printf("P: %s ------ \n      -- is Active %i -- Def: %i == Min: %i == Max: %i\n",
                       name.c_str(),
                       params.active(),
                       params.defaultValue(),
                       params.minimum(),
                       params.maximum());
            }
            return 0;
        }

        ///////////////////////////////////////////////////////////
        /// SET CAMERA PARAMETERS
        ///////////////////////////////////////////////////////////

        if(*EXPOSURE > -1)
        {
            input->setCaptureProperty(mCameraParameters.EXPOSURE_AUTO, 0);
            input->setCaptureProperty(mCameraParameters.EXPOSURE, *EXPOSURE);
            printf("EXPOSURE %i set.\n", *EXPOSURE);
        }
        if(*GAIN > -1)
        {
            input->setCaptureProperty(mCameraParameters.GAIN_AUTO, 0);
            input->setCaptureProperty(mCameraParameters.GAIN, *GAIN);
            printf("GAIN %i set.\n", *GAIN);
        }
        if(*WHITE_BALANCE > -1)
        {
            input->setCaptureProperty(mCameraParameters.AUTO_WHITE_BALANCE, 0);
            input->setCaptureProperty(mCameraParameters.WHITE_BALANCE, *WHITE_BALANCE);
            printf("WB %i set.\n", *WHITE_BALANCE);
        }

        input->startCapture();

        captures.push_back(input);
        frameToSkipList.push_back(&frameToSkip);
    }

    QTimer::singleShot(4000, &app, SLOT(quit()));
    return app.exec();
}
