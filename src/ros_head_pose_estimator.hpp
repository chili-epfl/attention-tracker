#include <string>
#include <set>

#include "head_pose_estimation.hpp"

// opencv2
#include <opencv2/core/core.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#define CV_INTER_LINEAR 1 // for compatibility with opencv 3
#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>


class HeadPoseEstimator
{
public:

    HeadPoseEstimator(ros::NodeHandle& rosNode,
                      const std::string& modelFilename = "");

private:

    ros::NodeHandle& rosNode;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber sub;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    image_geometry::PinholeCameraModel cameramodel;
    cv::Mat cameraMatrix, distCoeffs;

    cv::Mat inputImage;
    HeadPoseEstimation estimator;

    bool warnUncalibratedImage;

    void detectFaces(const sensor_msgs::ImageConstPtr& msg,
                     const sensor_msgs::CameraInfoConstPtr& camerainfo);
};

