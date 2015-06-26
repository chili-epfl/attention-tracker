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

    VideoCapture video_in(2);

    // settings for the Logitech c920
    video_in.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    video_in.set(CV_CAP_PROP_FRAME_HEIGHT, 360);

    estimator.focalLength = 455;
    estimator.opticalCenterX = 313;
    estimator.opticalCenterY = 167;

    if(!video_in.isOpened()) {
        cerr << "Couldn't open camera" << endl;
        return 1;
    }


    while(true) {
        video_in >> frame;

        estimator.update(frame);

        for(auto pose : estimator.poses()) {

        cout << "Head pose: (" << pose.x << ", " << pose.y << ", " << pose.z << ")" << endl;
        //cout << ", rx: " << pose.rx * 180 / M_PI << "deg, ry: " << pose.ry * 180 / M_PI << "deg, rz: " << pose.rz * 180 / M_PI << "deg" << endl;

        }

#ifdef HEAD_POSE_ESTIMATION_DEBUG
        imshow("headpose", estimator._debug);
        if (waitKey(10) >= 0) break;
#endif

    }
}



