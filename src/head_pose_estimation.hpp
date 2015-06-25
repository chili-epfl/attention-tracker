#ifndef __HEAD_POSE_ESTIMATION
#define __HEAD_POSE_ESTIMATION

#include <opencv2/core/core.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>

#include <vector>
#include <array>
#include <string>

// Anthropometric for male adult
// Values taken from https://en.wikipedia.org/wiki/Human_head
static const float DIST_SELLION_TO_STOMION= 199 - 124; //mm
static const float BITRAGION_BREADTH= 155; //mm


static const int MAX_FEATURES_TO_TRACK=100;

// Interesting facial features with their landmark index
enum FACIAL_FEATURE {
    NOSE=30,
    RIGHT_SIDE=0,
    LEFT_SIDE=16,
    EYEBROW_RIGHT=21,
    EYEBROW_LEFT=22,
    MOUTH_UP=51,
    MOUTH_DOWN=57,
    MOUTH_RIGHT=48,
    MOUTH_LEFT=54,
    SELLION=27,
    MOUTH_CENTER_TOP=62,
    MOUTH_CENTER_BOTTOM=66
};


struct head_pose {
    float x, y, z;
    float pitch, yaw;
};

class HeadPoseEstimation {

public:

    HeadPoseEstimation(const std::string& face_detection_model = "shape_predictor_68_face_landmarks.dat", float focalLength=650.);

    void update(cv::Mat image);

    bool smileDetector(size_t face_idx);

    float novelty(std::vector<bool> lookAt,
                std::vector<dlib::rectangle> faces, 
                float mu, float eps, float threshold);

    head_pose pose(size_t face_idx);

    std::vector<head_pose> poses();

    float quantityOfMovement(cv::Mat rgbFrames, 
                            cv::Mat grayFrames, 
                            cv::Mat prevGrayFrame,
                            cv::Mat opticalFlow, 
                            std::vector<cv::Point2f> &points1, 
                            std::vector<cv::Point2f> &points2, 
                            bool needToInit);

    float focalLength;
    float opticalCenterX;
    float opticalCenterY;

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    cv::Mat _debug;
#endif

private:

    dlib::cv_image<dlib::bgr_pixel> current_image;

    dlib::frontal_face_detector detector;
    dlib::shape_predictor pose_model;

    std::vector<dlib::rectangle> faces;

    std::vector<dlib::full_object_detection> shapes;


    /** Return the point corresponding to the dictionary marker.
    */
    cv::Point2f coordsOf(size_t face_idx, FACIAL_FEATURE feature);

    /** Returns true if the lines intersect (and set r to the intersection
     *  coordinates), false otherwise.
     */
    bool intersection(cv::Point2f o1, cv::Point2f p1,
                      cv::Point2f o2, cv::Point2f p2,
                      cv::Point2f &r);

};

#endif // __HEAD_POSE_ESTIMATION
