#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Range.h>

using namespace std;
int main( int argc, char** argv )
{
  ros::init(argc, argv, "expected_focus");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("expected_focus", 1);
  ros::Publisher fov_pub = n.advertise<sensor_msgs::Range>("child_field_of_view", 1);


  vector<string> frames = {"/tablet", "/experimenter", "/robot_head"};

  int i = 0;

  while (ros::ok())
  {
    
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = frames[i%3];
    i++;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "focus_of_attention";
    marker.id = 0;

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

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    
    // Field of view
    sensor_msgs::Range fov;

    fov.header.frame_id = "child_head";
    fov.header.stamp = ros::Time::now();
    fov.radiation_type = sensor_msgs::Range::INFRARED;
    fov.field_of_view = 0.4; // ~20deg
    fov.min_range = 0.1;
    fov.max_range = 1;
    fov.range = 0.6;

    fov_pub.publish(fov); 


    r.sleep();
  }
}
