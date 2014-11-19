#include <iostream>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include <libfreenect2/opengl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>

using namespace std;

bool shutdown = false;

void sigint_handler(int s)
{
    shutdown = true;
}

int main(int argc, char *argv[]) {

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = freenect2.openDefaultDevice();

    if(dev == 0)
    {
        std::cout << "no device connected or failure opening the default one!" << std::endl;
        return -1;
    }

    signal(SIGINT,sigint_handler);
    shutdown = false;

    // set up the parameters for blob detector (check the defaults in opencv's code in blobdetector.cpp)
    cv::SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 50.0f;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.filterByCircularity = false;
    params.filterByArea = true;
    params.minArea = 20.0f;
    params.maxArea = 500.0f;
// ... any other params you don't want default value

// set up and create the detector using the parameters
//this won't build for some reason
    cv::SimpleBlobDetector blob_detector(params);

    blob_detector.create("SimpleBlob");


    // setup kinect frame listener (from protonect example)
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    while(!shutdown) {
        //get frames from kinect
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
        cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
        cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);

        //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
		
//TODO: CONVERT Frame *depth info into an image that openCV can use

        // detect!
        vector<cv::KeyPoint> keypoints;
        blob_detector.detect(image, keypoints);

        // extract the x y coordinates of the keypoints:
        for (int i = 0; i < keypoints.size(); i++) {
            float X = keypoints[i].pt.x;
            float Y = keypoints[i].pt.y;
        }


        int key = cv::waitKey(1);
        shutdown = shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
        listener.release(frames);
    }
    dev->stop();
    dev->close();

    return 0;

}
