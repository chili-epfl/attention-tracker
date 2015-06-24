#include <string>
#include <ros/ros.h>

#include "ros_head_pose_estimator.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "ros_markers");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");

    // load parameters
    string modelFilename;
    _private_node.param<string>("face_model", modelFilename, "");

    if (modelFilename.empty()) {
        ROS_ERROR_STREAM("You must provide the face model with the parameter face_model.\n" <<
                         "For instance, _face_model:=shape_predictor_68_face_landmarks.dat");
        return(1);
    }

    // initialize the detector by subscribing to the camera video stream
    ROS_INFO_STREAM("Initializing the face detector with the model " << modelFilename <<"...");
    HeadPoseEstimator estimator(rosNode, modelFilename);
    ROS_INFO("head_pose_estimator is ready. TF frames of detected faces will be published when detected.");
    ros::spin();

    return 0;
}

