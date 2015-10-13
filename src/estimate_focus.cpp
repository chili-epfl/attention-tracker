#include <string>
#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

using namespace std;

static const string HUMAN_FRAME_PREFIX = "face_";

static const double FOV = 30. / 180 * M_PI; // radians
static const float RANGE = 3; //m

std::vector<std_msgs::ColorRGBA> colors;
static std_msgs::ColorRGBA GREEN;
static std_msgs::ColorRGBA BLUE;
static std_msgs::ColorRGBA RED;


visualization_msgs::Marker makeMarker(int id, const string& frame, std_msgs::ColorRGBA color) {

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "focus_of_attention";
    marker.id = id;

    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    marker.color = color;

    marker.lifetime = ros::Duration(0.5);

    return marker;
}

bool isInFieldOfView(const tf::TransformListener& listener, const string& target_frame, const string& observer_frame) {

    tf::StampedTransform transform;

    try{
        listener.lookupTransform(observer_frame, target_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // object behind the observer?
    if (transform.getOrigin().x() < 0) return false;

    // the field of view's main axis is the observer's X axis. So, distance to main axis is
    // simply sqrt(y^2 + z^2)
    double distance_to_main_axis = sqrt(transform.getOrigin().y() * transform.getOrigin().y() + transform.getOrigin().z() * transform.getOrigin().z());

    double fov_radius_at_x = tan(FOV/2) * transform.getOrigin().x();

    //ROS_INFO_STREAM(target_frame << ": distance_to_main_axis: " << distance_to_main_axis << ", fov_radius_at_x: " << fov_radius_at_x);
    if (distance_to_main_axis < fov_radius_at_x) return true;

    return false;

}

int main( int argc, char** argv )
{
    GREEN.r = 0.; GREEN.g = 1.; GREEN.b = 0.; GREEN.a = 1.;
    BLUE.r = 0.; BLUE.g = 0.; BLUE.b = 1.; BLUE.a = 1.;
    RED.r = 1.; RED.g = 0.; RED.b = 0.; RED.a = 1.;
    colors.push_back(GREEN);
    colors.push_back(BLUE);
    colors.push_back(RED);

    ros::init(argc, argv, "estimate_focus");
    ros::NodeHandle n;
    ros::Rate r(30);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("estimate_focus", 1);
    ros::Publisher fov_pub = n.advertise<sensor_msgs::Range>("face_0_field_of_view", 1);
    ros::Publisher frames_in_fov_pub = n.advertise<std_msgs::String>("actual_focus_of_attention", 1);

    tf::TransformListener listener;
    vector<string> frames;

    vector<string> monitored_frames = {"/robot_head", "/tablet", "/selection_tablet", "/experimenter"};


    // Prepare a range sensor msg to represent the fields of view
    // (cone for visualization)
    sensor_msgs::Range fov;

    fov.radiation_type = sensor_msgs::Range::INFRARED;
    fov.field_of_view = FOV;
    fov.min_range = 0;
    fov.max_range = 10;
    fov.range = RANGE;

  ROS_INFO("Waiting until a face becomes visible...");
  while (!listener.waitForTransform("base_footprint", "face_0", ros::Time::now(), ros::Duration(5.0))) {
        ROS_DEBUG("Still no face visible...");
        r.sleep();
  }

  ROS_INFO("Face detected! We can start estimating the focus of attention...");

  while (ros::ok())
  {
    
    // Publish the marker
    /*while (marker_pub.getNumSubscribers() < 1 && fov_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker or field of view");
      sleep(1);
    }*/

    frames.clear();
    listener.getFrameStrings(frames);
    int nb_faces = 0;

    for(auto frame : frames) {
        if(frame.find(HUMAN_FRAME_PREFIX) == 0) {

            nb_faces++;

            int face_idx = stoi(frame.substr(HUMAN_FRAME_PREFIX.length(), 1));

            if (face_idx == 0) {
                
                stringstream ss;
                for(size_t i = 0 ; i < monitored_frames.size(); ++i) {
                    if(isInFieldOfView(listener, monitored_frames[i], frame)) {
                        ROS_DEBUG_STREAM(monitored_frames[i] << " is in the field of view of " << frame);
                        marker_pub.publish(makeMarker(i, monitored_frames[i], colors[i]));
                        // create new marker with special color for the observed frame ? 
                        if (ss.str().empty()) ss << " ";
                        ss << monitored_frames[i];
                        //ss is the frame that the observer is looking
                    }
                }
                frames_in_fov_pub.publish(ss.str());

                fov.range = RANGE;
                fov.header.stamp = ros::Time::now();
                fov.header.frame_id = frame;
                fov_pub.publish(fov); 
            }
        }
    }
    if (nb_faces == 0) {

        // hide the field of view
        fov.range = 0;

        fov.header.stamp = ros::Time::now();
        fov.header.frame_id = "face_0";
        fov_pub.publish(fov);
        
        //stringstream ss;
        //ss << "test";
        //frames_in_fov_pub.publish(ss.str());

    }
    r.sleep();
  }
}
