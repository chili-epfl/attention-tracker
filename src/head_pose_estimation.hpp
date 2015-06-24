#ifndef __HEAD_POSE_ESTIMATION
#define __HEAD_POSE_ESTIMATION

#include <opencv2/core/core.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>

#include <vector>
#include <string>

static const int MAX_FEATURES_TO_TRACK=100;

struct head_pose {
    float x, y, z;
    float pitch, yaw;
};

class HeadPoseEstimation {

public:

    // Create a dictionary for the markers.
    std::map<std::string, int> partToPoint = {
        {"nose", 30},
        {"right_side", 2},
        {"left_side", 14},
        {"eyebrow_right", 21},
        {"eyebrow_left", 22},
        {"mouth_up", 51},
        {"mouth_down", 57},
        {"mouth_right", 48},
        {"mouth_left", 54}
    };

    HeadPoseEstimation(const std::string& face_detection_model = "shape_predictor_68_face_landmarks.dat");

    void update(cv::Mat image);

    int headSize(size_t face_idx);

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
    cv::Point2f getPointFromPart(size_t face_idx, 
                                 std::string name);

    /** Returns true if the lines intersect, false otherwise.
    */
    bool getLineIntersection(float p0_x, float p0_y, float p1_x, float p1_y,
                            float p2_x, float p2_y, float p3_x, float p3_y);

};

#endif // __HEAD_POSE_ESTIMATION
