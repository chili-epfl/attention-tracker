#include <cmath>
#include <ctime>

#include <opencv2/calib3d/calib3d.hpp>

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


void HeadPoseEstimation::update(cv::InputArray _image)
{

    Mat image = _image.getMat();

    if (opticalCenterX == -1) // not initialized yet
    {
        opticalCenterX = image.cols / 2;
        opticalCenterY = image.rows / 2;
#ifdef HEAD_POSE_ESTIMATION_DEBUG
        cerr << "Setting the optical center to (" << opticalCenterX << ", " << opticalCenterY << ")" << endl;
#endif
    }

    current_image = cv_image<bgr_pixel>(image);

    faces = detector(current_image,_UPSAMPLE);

    // Find the pose of each face.
    shapes.clear();
    for (auto face : faces){
        shapes.push_back(pose_model(current_image, face));
    }

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    // Draws the contours of the face and face features onto the image
    
    _debug = image.clone();

    auto color = Scalar(0,128,128);

    for (unsigned long i = 0; i < shapes.size(); ++i)
    {
        const full_object_detection& d = shapes[i];

        for (unsigned long i = 1; i <= 16; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);

        for (unsigned long i = 28; i <= 30; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);

        for (unsigned long i = 18; i <= 21; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        for (unsigned long i = 23; i <= 26; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        for (unsigned long i = 31; i <= 35; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(30)), toCv(d.part(35)), color, 2, CV_AA);

        for (unsigned long i = 37; i <= 41; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(36)), toCv(d.part(41)), color, 2, CV_AA);

        for (unsigned long i = 43; i <= 47; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(42)), toCv(d.part(47)), color, 2, CV_AA);

        for (unsigned long i = 49; i <= 59; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(48)), toCv(d.part(59)), color, 2, CV_AA);

        for (unsigned long i = 61; i <= 67; ++i)
            line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
        line(_debug, toCv(d.part(60)), toCv(d.part(67)), color, 2, CV_AA);

        for (auto i = 0; i < 68 ; i++) {
            putText(_debug, to_string(i), toCv(d.part(i)), FONT_HERSHEY_DUPLEX, 0.6, Scalar(255,255,255));
        }
    }
#endif
}

head_pose HeadPoseEstimation::pose(size_t face_idx) const
{

    cv::Mat projectionMat = cv::Mat::zeros(3,3,CV_32F);
    cv::Matx33f projection = projectionMat;
    projection(0,0) = focalLength;
    projection(1,1) = focalLength;
    projection(0,2) = opticalCenterX;
    projection(1,2) = opticalCenterY;
    projection(2,2) = 1;

    std::vector<Point3f> head_points;

    head_points.push_back(P3D_SELLION);
    head_points.push_back(P3D_RIGHT_EYE);
    head_points.push_back(P3D_LEFT_EYE);
    head_points.push_back(P3D_RIGHT_EAR);
    head_points.push_back(P3D_LEFT_EAR);
    head_points.push_back(P3D_MENTON);
    head_points.push_back(P3D_NOSE);
    head_points.push_back(P3D_STOMMION);

    std::vector<Point2f> detected_points;

    detected_points.push_back(coordsOf(face_idx, SELLION));
    detected_points.push_back(coordsOf(face_idx, RIGHT_EYE));
    detected_points.push_back(coordsOf(face_idx, LEFT_EYE));
    detected_points.push_back(coordsOf(face_idx, RIGHT_SIDE));
    detected_points.push_back(coordsOf(face_idx, LEFT_SIDE));
    detected_points.push_back(coordsOf(face_idx, MENTON));
    detected_points.push_back(coordsOf(face_idx, NOSE));

    auto stomion = (coordsOf(face_idx, MOUTH_CENTER_TOP) + coordsOf(face_idx, MOUTH_CENTER_BOTTOM)) * 0.5;
    detected_points.push_back(stomion);

    Mat rvec, tvec;

    // Find the 3D pose of our head
    solvePnP(head_points, detected_points,
            projection, noArray(),
            rvec, tvec, false,
#ifdef OPENCV3
            cv::SOLVEPNP_ITERATIVE);
#else
            cv::ITERATIVE);
#endif

    Matx33d rotation;
    Rodrigues(rvec, rotation);


    head_pose pose = {
        rotation(0,0),    rotation(0,1),    rotation(0,2),    tvec.at<double>(0)/1000,
        rotation(1,0),    rotation(1,1),    rotation(1,2),    tvec.at<double>(1)/1000,
        rotation(2,0),    rotation(2,1),    rotation(2,2),    tvec.at<double>(2)/1000,
                    0,                0,                0,                     1
    };

#ifdef HEAD_POSE_ESTIMATION_DEBUG

    std::vector<Point2f> reprojected_points;

    projectPoints(head_points, rvec, tvec, projection, noArray(), reprojected_points);

    for (auto point : reprojected_points) {
        circle(_debug, point,2, Scalar(0,255,255),2);
    }

    std::vector<Point3f> axes;
    axes.push_back(Point3f(0,0,0));
    axes.push_back(Point3f(50,0,0));
    axes.push_back(Point3f(0,50,0));
    axes.push_back(Point3f(0,0,50));
    std::vector<Point2f> projected_axes;

    projectPoints(axes, rvec, tvec, projection, noArray(), projected_axes);

    line(_debug, projected_axes[0], projected_axes[3], Scalar(255,0,0),2,CV_AA);
    line(_debug, projected_axes[0], projected_axes[2], Scalar(0,255,0),2,CV_AA);
    line(_debug, projected_axes[0], projected_axes[1], Scalar(0,0,255),2,CV_AA);

    putText(_debug, "(" + to_string(int(pose(0,3) * 100)) + "cm, " + to_string(int(pose(1,3) * 100)) + "cm, " + to_string(int(pose(2,3) * 100)) + "cm)", coordsOf(face_idx, SELLION), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2);


#endif

    return pose;
}

std::vector<head_pose> HeadPoseEstimation::poses() const {

    std::vector<head_pose> res;

    for (auto i = 0; i < faces.size(); i++){
        res.push_back(pose(i));
    }

    return res;

}

Point2f HeadPoseEstimation::coordsOf(size_t face_idx, FACIAL_FEATURE feature) const
{
    return toCv(shapes[face_idx].part(feature));
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
// taken from: http://stackoverflow.com/a/7448287/828379
bool HeadPoseEstimation::intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
                                      Point2f &r) const
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

