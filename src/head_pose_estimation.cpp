#include <cmath>
#include <ctime>

#ifdef HEAD_POSE_ESTIMATION_DEBUG
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#endif

#include "head_pose_estimation.hpp"

using namespace dlib;
using namespace std;
using namespace cv;

inline Point2f toCv(const dlib::point& p)
{
    return Point2f(p.x(), p.y());
}


HeadPoseEstimation::HeadPoseEstimation(const string& face_detection_model, float focalLength) :
        focalLength(focalLength),
        opticalCenterX(-1),
        opticalCenterY(-1)
{

        // Load face detection and pose estimation models.
        detector = get_frontal_face_detector();
        deserialize(face_detection_model) >> pose_model;

}


void HeadPoseEstimation::update(cv::Mat image)
{

    if (opticalCenterX == -1) // not initialized yet
    {
        opticalCenterX = image.cols / 2;
        opticalCenterY = image.rows / 2;
    }

    current_image = cv_image<bgr_pixel>(image);

    faces = detector(current_image);

    // Find the pose of each face.
    shapes.clear();
    for (auto face : faces){
        shapes.push_back(pose_model(current_image, face));
    }

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    // Draws the contours of the face and face features onto the image
    
    _debug = image.clone();

    auto color = Scalar(0,255,128);

    for (unsigned long i = 0; i < shapes.size(); ++i)
    {
        const full_object_detection& d = shapes[i];

        for (auto i = 0; i < 68 ; i++) {
            putText(_debug, to_string(i), toCv(d.part(i)), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));
        }
        for (unsigned long i = 1; i <= 16; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, LINE_AA);

        for (unsigned long i = 28; i <= 30; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, LINE_AA);

        for (unsigned long i = 18; i <= 21; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, LINE_AA);
        for (unsigned long i = 23; i <= 26; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, LINE_AA);
        for (unsigned long i = 31; i <= 35; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, LINE_AA);
        line(_debug, toCv(d.part(30)), toCv(d.part(35)), color, 2, LINE_AA);

        for (unsigned long i = 37; i <= 41; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, LINE_AA);
        line(_debug, toCv(d.part(36)), toCv(d.part(41)), color, 2, LINE_AA);

        for (unsigned long i = 43; i <= 47; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, LINE_AA);
        line(_debug, toCv(d.part(42)), toCv(d.part(47)), color, 2, LINE_AA);

        for (unsigned long i = 49; i <= 59; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, LINE_AA);
        line(_debug, toCv(d.part(48)), toCv(d.part(59)), color, 2, LINE_AA);

        for (unsigned long i = 61; i <= 67; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, LINE_AA);
        line(_debug, toCv(d.part(60)), toCv(d.part(67)), color, 2, LINE_AA);
    }
#endif
}


head_pose HeadPoseEstimation::pose(size_t face_idx)
{

    head_pose pose;

    // facial features coordinates used for head pose estimation
    auto right = coordsOf(face_idx, RIGHT_SIDE);
    auto left = coordsOf(face_idx, LEFT_SIDE);
    auto sellion = coordsOf(face_idx, SELLION);
    auto stomion = (coordsOf(face_idx, MOUTH_CENTER_TOP) + coordsOf(face_idx, MOUTH_CENTER_BOTTOM)) / 2;

    // head dimensions (in pixels!)
    float width = norm(right - left);
    float height = norm(sellion - stomion);

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    line(_debug, right, left, Scalar(0,255,255), 1, LINE_AA);
    line(_debug, sellion, stomion, Scalar(0,255,255), 1, LINE_AA);
    putText(_debug, to_string(width), (left + right) / 2, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,0));
    putText(_debug, to_string(height), (sellion + stomion) / 2, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,0));
#endif

    // compute the head's distance in mm as the max of a distance computed with the
    // head breadth and one computed with the face height We take the max as an
    // easy way to compensate for either pitch or yaw rotations
    auto head_distance = max(BITRAGION_BREADTH * focalLength / width,
                             DIST_SELLION_TO_STOMION * focalLength / height);


    Point2f width_heigth_intersection;
    intersection(right, left, sellion, stomion, width_heigth_intersection);
#ifdef HEAD_POSE_ESTIMATION_DEBUG
    circle(_debug, width_heigth_intersection,2, Scalar(0,0,255));
#endif


    // yaw
    float width_right = norm(right - width_heigth_intersection); 
    float width_left = norm(left - width_heigth_intersection); 
    auto east_west = (width_right - width_left) / width; // -1<score<1
    auto yaw = M_PI/3 * east_west + M_PI;

    // pitch
    auto south_north = norm(stomion - width_heigth_intersection) / height;

    // normalize: 0.9 is the 'south_north' ratio when facing the camera
    // then, on my face, the ratio value goes from ~0.4 to ~1.4
    south_north = -std::min(1., std::max(-1.,(south_north - 0.9) / 0.5));
    auto pitch = M_PI/4 * south_north;

    pose.x = (width_heigth_intersection.x - opticalCenterX) * head_distance / focalLength;
    pose.y = (width_heigth_intersection.y - opticalCenterY) * head_distance / focalLength;
    pose.z = head_distance;
    pose.pitch = pitch;
    pose.yaw = yaw;

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    putText(_debug, "(" + to_string(int(pose.x/10)) + "cm, " + to_string(int(pose.y/10)) + "cm, " + to_string(int(pose.z/10)) + "cm)", width_heigth_intersection, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255),2);
#endif


    return pose;
}

std::vector<head_pose> HeadPoseEstimation::poses() {

    std::vector<head_pose> res;

    for (auto i = 0; i < faces.size(); i++){
        res.push_back(pose(i));
    }

    return res;

}

Point2f HeadPoseEstimation::coordsOf(size_t face_idx, FACIAL_FEATURE feature)
{
    return toCv(shapes[face_idx].part(feature));
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
// taken from: http://stackoverflow.com/a/7448287/828379
bool HeadPoseEstimation::intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
                                      Point2f &r)
{
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

