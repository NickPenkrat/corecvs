#include <stdio.h>
#include <QtCore/QCoreApplication>
#include <QtCore/QThread>
#ifndef WIN32
#include <unistd.h>
#endif


#include "imageCaptureInterface.h"
//#include "V4L2Capture.h"
#include "bmpLoader.h"

int main (int argc, char **argv)
{
	QCoreApplication app(argc, argv);
	printf("Attempting a grab\n");
    //V4L2CaptureInterface *input = static_cast<V4L2CaptureInterface*>(ImageCaptureInterface::fabric("v4l2:/dev/video0,/dev/video1"));
    ImageCaptureInterface *input = ImageCaptureInterface::fabric("v4l2:/dev/video0,/dev/video1");


    input->initCapture();
	input->startCapture();

//    QThread::sleep(2);
//	sleep(2);

	RGB24Buffer *result = NULL;

	for(int i = 0; i < 40; i++) {
	    delete_safe(result);
        //V4L2CaptureInterface::FramePair pair = input->getFrameRGB24();
        ImageCaptureInterface::FramePair pair = input->getFrameRGB24();
	    result = pair.rgbBufferLeft;
	    delete_safe(pair.bufferRight);
	    delete_safe(pair.bufferLeft);
        delete_safe(pair.rgbBufferRight);
	}

	BMPLoader().save("snapshot.bmp", result);
	delete_safe(result);
	return 0;
}
