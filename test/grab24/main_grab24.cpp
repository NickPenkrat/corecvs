#include <stdio.h>
#include <QtCore/QCoreApplication>
#include <QtCore/QThread>
#include <unistd.h>


#include "imageCaptureInterface.h"
#include "V4L2Capture.h"
#include "bmpLoader.h"

int main (int argc, char **argv)
{
	QCoreApplication app(argc, argv);
	printf("Attempting a grab\n");
	V4L2CaptureInterface *input = static_cast<V4L2CaptureInterface*>(ImageCaptureInterface::fabric("v4l2:/dev/video0,/dev/video1"));
	input->initCapture();
	input->startCapture();

	sleep(2);

	RGB24Buffer *result = NULL;

	for(int i = 0; i < 20; i++) {
	    delete_safe(result);
	    result = input->getFrameRGB24();
	}

	BMPLoader().save("snapshot.bmp", result);
	delete_safe(result);
	return 0;
}
