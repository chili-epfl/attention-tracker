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
        focalLength(focalLength)
{

        // Load face detection and pose estimation models.
        detector = get_frontal_face_detector();
        deserialize(face_detection_model) >> pose_model;

}


void HeadPoseEstimation::update(cv::Mat image)
{
    current_image = image;

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


std::array<float,2> HeadPoseEstimation::headDimensions(size_t face_idx)
{
    auto right = coordsOf(face_idx, RIGHT_SIDE);
    auto left = coordsOf(face_idx, LEFT_SIDE);

    float width = sqrt((right.x-left.x)*(right.x-left.x) + (right.y-left.y)*(right.y-left.y));

    auto sellion = coordsOf(face_idx, SELLION);
    auto stomion = (coordsOf(face_idx, MOUTH_CENTER_TOP) + coordsOf(face_idx, MOUTH_CENTER_BOTTOM)) / 2;

    float height = sqrt((sellion.x-stomion.x)*(sellion.x-stomion.x) + (sellion.y-stomion.y)*(sellion.y-stomion.y));

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    line(_debug, right, left, Scalar(0,255,255), 1, LINE_AA);
    line(_debug, sellion, stomion, Scalar(0,255,255), 1, LINE_AA);
    putText(_debug, to_string(width), (left + right) / 2, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,0));
    putText(_debug, to_string(height), (sellion + stomion) / 2, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,0));
#endif

    return {width, height};

}

head_pose HeadPoseEstimation::pose(size_t face_idx)
{

    head_pose pose;

    auto head_size = headDimensions(face_idx);

    // compute the head's distance as the max of a distance computed with the head breadth and one computed with the face height
    // We take the max as an easy way to compensate for either pitch or yaw rotations
    auto head_distance = max(BITRAGION_BREADTH * focalLength / head_size[0],
                             DIST_SELLION_TO_STOMION * focalLength / head_size[1]);

    // Get nose
    Point2f nose = coordsOf(face_idx, NOSE);

    /*float nose.x = pose_model(current_image, faces[i]).part(30).x();
    float nose.y = pose_model(current_image, faces[i]).part(30).y();
    noses.push_back(centered_rect(point(nose.x,nose.y),8,8));*/

    //get rights
    Point2f right = coordsOf(face_idx, RIGHT_SIDE);
    //get lefts
    Point2f left = coordsOf(face_idx, LEFT_SIDE);

    float horRight = sqrt((right.x-nose.x)*(right.x-nose.x) + (right.y-nose.y)*(right.y-nose.y));
    float horLeft = sqrt((left.x-nose.x)*(left.x-nose.x) + (left.y-nose.y)*(left.y-nose.y));
    float horizontal = sqrt((right.x-left.x)*(right.x-left.x) + (right.y-left.y)*(right.y-left.y));
    auto east_west = (horRight-horLeft)/horizontal; // -1<score<1

    float l = horRight*horizontal/(horRight+horLeft);
    float xl = right.x+(left.x-right.x)*l/horizontal;
    float yl = right.y+(left.y-right.y)*l/horizontal;

    float sh = (yl-nose.y)/abs(nose.y-yl);
    float h = sqrt((nose.y-yl)*(nose.y-yl) + (nose.x-xl)*(nose.x-xl));
    float south_north = sh*h/horRight;


    // compute "discrete" gaze directions
    // TODO: not used for now!
    std::vector<bool> lookAt(4);
    lookAt[1] = east_west>0.3;
    lookAt[0] = east_west<-0.3;
    lookAt[2] = south_north>0.3;
    lookAt[3] = south_north<-0.3;

    //hack for southnord between -1 and 1 :
    if(south_north>0){
        south_north = south_north/0.45;
    }
    if(south_north<0){
        south_north = south_north/0.37;
    }

    //convert values to have measure of angles:
    south_north = (M_PI/4)*south_north - M_PI/2;
    east_west = (M_PI/3)*east_west;

    //get projection (if we want to plot on the window) :
    //auto x_gaze = sin(east_west)*len/20;
    //auto y_gaze = sin(south_north)*wid/20;

    pose.x = head_distance * nose.x / focalLength;
    pose.y = head_distance * nose.y / focalLength;
    pose.z = head_distance;
    pose.pitch = south_north;
    pose.yaw = east_west;

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

bool HeadPoseEstimation::getLineIntersection(float p0_x, float p0_y, float p1_x, float p1_y,
                           float p2_x, float p2_y, float p3_x, float p3_y){

    float s1_x, s1_y, s2_x, s2_y;

    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;

    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1){
        return true;
    }
    return false;
}


