#ifdef OPENCV3
#include <opencv2/imgcodecs.hpp>
#else
#include <opencv2/highgui/highgui.hpp>
#endif

#include <iostream>

#include "../src/head_pose_estimation.hpp"

using namespace std;
using namespace cv;

const static size_t NB_TESTS = 100; // number of time the detection is run, to get better average detection duration

int main(int argc, char **argv)
{
    Mat frame;

    if(argc < 3) {
        cerr << "Usage: " << endl <<
                "head_pose model.dat frame.{jpg|png}" << endl;
#ifdef HEAD_POSE_ESTIMATION_DEBUG
        cerr <<  "Output: a new frame 'head_pose_<frame>.png'" << endl;
#endif
        return 1;
    }


    auto estimator = HeadPoseEstimation(argv[1]);

    cout << "Estimating head pose on " << argv[2] << endl;
    Mat img = imread(argv[2],CV_LOAD_IMAGE_COLOR);

    estimator.focalLength = 500;


    cout << "Running " << NB_TESTS << " loops to get a good performance estimate..." << endl;
#ifdef HEAD_POSE_ESTIMATION_DEBUG
    cerr <<  "ATTENTION! The benchmark is compiled in DEBUG mode: the performance is no going to be good!!" << endl;
#endif
    auto t_start = getTickCount();

    auto nbfaces = 0;

    for(size_t i = 0; i < NB_TESTS; i++) {
        estimator.update(img);
    }
    auto t_detection = getTickCount();

    for(size_t i = 0; i < NB_TESTS; i++) {
        auto poses = estimator.poses();
        nbfaces += poses.size(); // this is completly artifical: the only purpose is to make sure the compiler does not optimize away estimator.poses()
    }
    auto t_end = getTickCount();

    cout << "Found " << nbfaces/NB_TESTS << " faces" << endl;

    auto poses = estimator.poses();
    for(auto pose : poses) {
    cout << "Head pose: (" << pose(0,3) << ", " << pose(1,3) << ", " << pose(2,3) << ")" << endl;

    }

    cout << "Face feature detection: " <<((t_detection-t_start) / NB_TESTS) /getTickFrequency() * 1000. << "ms;";
    cout << "Pose estimation: " <<((t_end-t_detection) / NB_TESTS) /getTickFrequency() * 1000. << "ms;";
    cout << "Total time: " << ((t_end-t_start) / NB_TESTS) / getTickFrequency() * 1000. << "ms" << endl;

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    imwrite("head_pose.png", estimator._debug);
#endif

}


