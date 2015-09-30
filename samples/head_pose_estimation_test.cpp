#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "../src/head_pose_estimation.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    Mat frame;

    namedWindow("headpose");

    if(argc < 2) {
        cerr << "Usage: " << endl <<
                "head_pose model.dat" << endl;
        return 1;
    }


    auto estimator = HeadPoseEstimation(argv[1]);

    // Configure the video capture
    // ===========================

    VideoCapture video_in(0);

    // adjust for your webcam!
    video_in.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    video_in.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    estimator.focalLength = 500;
    estimator.opticalCenterX = 320;
    estimator.opticalCenterY = 240;

    if(!video_in.isOpened()) {
        cerr << "Couldn't open camera" << endl;
        return 1;
    }


    while(true) {
        video_in >> frame;

        auto t_start = getTickCount();
        estimator.update(frame);

        for(auto pose : estimator.poses()) {

        cout << "Head pose: (" << pose(0,3) << ", " << pose(1,3) << ", " << pose(2,3) << ")" << endl;
        auto t_end = getTickCount();
        cout << "Processing time for this frame: " << (t_end-t_start) / getTickFrequency() * 1000. << "ms" << endl;

        }

#ifdef HEAD_POSE_ESTIMATION_DEBUG
        imshow("headpose", estimator._debug);
        if (waitKey(10) >= 0) break;
#endif

    }
}



