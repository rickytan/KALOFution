#include "MP4Dumper.h"
#include "Depth2PLY.h"

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <pcl/io/ply_io.h>

MP4Dumper::MP4Dumper(const std::string& mp4file)
{
    m_videoCapture = cvCaptureFromFile(mp4file.c_str());
}


MP4Dumper::~MP4Dumper()
{
    cvFree(&m_videoCapture);
}

void MP4Dumper::dumpTo(const std::string& dir)
{
    while (cvGrabFrame(m_videoCapture)) {
        IplImage *frame = cvRetrieveFrame(m_videoCapture);
        cvShowImage("Video", frame);
        cvWaitKey(30);

        static int frameCount = 0;
        Depth2PLY depth2ply;
        CloudTypePtr cloud = depth2ply(frame->width, frame->height, frame->imageData, frame->widthStep);
        char filepath[1024] = { 0 };
        sprintf(filepath, "%s/cloud_%d.ply", dir.c_str(), frameCount++);
        pcl::io::savePLYFileBinary(filepath, *cloud);
    }
}
