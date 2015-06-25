#include "ros_head_pose_estimator.hpp"

using namespace std;
using namespace cv;

// how many second in the *future* the face transformation should be published?
// this allow to compensate for the 'slowness' of face detection, but introduce
// some lag in TF.
#define TRANSFORM_FUTURE_DATING 0

HeadPoseEstimator::HeadPoseEstimator(ros::NodeHandle& rosNode,
                                     const string& modelFilename) :
            rosNode(rosNode),
            it(rosNode),
            warnUncalibratedImage(true),
            estimator(modelFilename)

{
    sub = it.subscribeCamera("image", 1, &HeadPoseEstimator::detectFaces, this);

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    pub = it.advertise("attention_tracker/faces/image",1);
#endif
}

void HeadPoseEstimator::detectFaces(const sensor_msgs::ImageConstPtr& msg, 
                                    const sensor_msgs::CameraInfoConstPtr& camerainfo)
{
    // updating the camera model is cheap if not modified
    cameramodel.fromCameraInfo(camerainfo);
    // publishing uncalibrated images? -> return (according to CameraInfo message documentation,
    // K[0] == 0.0 <=> uncalibrated).
    if(cameramodel.intrinsicMatrix()(0,0) == 0.0) {
        if(warnUncalibratedImage) {
            warnUncalibratedImage = false;
            ROS_ERROR("Camera publishes uncalibrated images. Can not estimate face position.");
            ROS_WARN("Detection will start over again when camera info is available.");
        }
        return;
    }
    warnUncalibratedImage = true;
    
    estimator.focalLength = cameramodel.fx(); 
    estimator.opticalCenterX = cameramodel.cx();
    estimator.opticalCenterY = cameramodel.cy();

    // hopefully no copy here:
    //  - assignement operator of cv::Mat does not copy the data
    //  - toCvShare does no copy if the default (source) encoding is used.
    inputImage = cv_bridge::toCvShare(msg)->image; 

    /********************************************************************
    *                      Faces detection                           *
    ********************************************************************/

    estimator.update(inputImage);

    auto poses = estimator.poses();
    ROS_INFO_STREAM(poses.size() << " faces detected.");

    for(size_t face_idx = 0; face_idx < poses.size(); ++face_idx) {

        auto pose = poses[face_idx];

        tf::StampedTransform face_pose;

        face_pose.frame_id_ = cameramodel.tfFrame();
        face_pose.child_frame_id_ = "face_" + to_string(face_idx);
        face_pose.stamp_ = ros::Time::now() + ros::Duration(TRANSFORM_FUTURE_DATING);

        tf::Quaternion q;


        // Frame orientation of the camera follows the ROS
        // convention (x forward, y left, z up) and *not* the classical camera
        // convention (z forward)
        //q.setRPY(0., -pose.pitch, pose.yaw);
        //face_pose.setOrigin(tf::Vector3(pose.z/1000, -pose.x/1000, -pose.y/1000));

        // Frame orientation of the camera follows the classical camera
        // convention (Z forward)
        q.setRPY(0., -pose.pitch + M_PI/2, pose.yaw);
        face_pose.setOrigin(tf::Vector3(pose.x/1000, pose.y/1000, pose.z/1000));

        face_pose.setRotation(q);
        br.sendTransform(face_pose);

    }

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    if(pub.getNumSubscribers() > 0) {
        ROS_INFO_ONCE("Starting to publish face tracking output for debug");
        auto debugmsg = cv_bridge::CvImage(msg->header, "bgr8", estimator._debug).toImageMsg();
        pub.publish(debugmsg);
    }
#endif
}

