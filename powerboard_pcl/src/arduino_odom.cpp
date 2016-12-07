#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

// Uses longboard model described here:
// https://www.researchgate.net/profile/Alexander_Kuleshov/publication/225511625_Mathematical_model_of_a_skateboard_with_one_degree_of_freedom/links/0f317539f0c368a8d7000000.pdf?origin=publication_detail
// Note, wheel rpm sensors cannot discern direction of motor rotation, so backward and forward motion will both be read as forward

double time_prev = 0.0;
ros::Publisher odom_pub;
tf::TransformBroadcaster *odom_broadcaster;
//assuming 2D navigation; angles in radians
//assuming user is not standing on or riding board before first message is received
double imu_roll = 0.0;
double imu_pitch = 0.0;
double imu_yaw = 0.0; 
double roll_offset = 0.0;
double pitch_offset = 0.0;
double yaw_offset = 0.0;

bool imu_offset_flag = false;
double wheel_yaw, wheel_x, wheel_y; //angle in radians; 0 yaw assumed parallel to x-axis
double dim_a, dim_d;
std::string child_frame_id, parent_frame_id;

void imu_cb (const sensor_msgs::Imu& imu_msg)
{
  tf::Quaternion q(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(imu_roll, imu_pitch, imu_yaw);
  if (!imu_offset_flag){
    //initialize rpy offset
    m.getRPY(roll_offset, pitch_offset, yaw_offset);
    imu_offset_flag = true;
    ROS_INFO("Initial Offset! Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll_offset, pitch_offset, yaw_offset);
  }
  imu_roll-=roll_offset;
  imu_pitch-=pitch_offset;
  imu_yaw-=yaw_offset; //because IMU initializes yaw to 0, expect offset to be 0 as well
}

void wheel_cb (const geometry_msgs::Vector3& wheel_msg)
{
  // ROS_INFO("Wheel encoder message received")
  if (!((time_prev==0.0) || (time_prev > wheel_msg.z))){ //if time_prev not yet initialized or timer has wrapped around
    double delta = imu_roll*tan(dim_d);
    double u = (wheel_msg.x + wheel_msg.y)/2.0;
    double yaw_rate = ( u*sin(2*delta) ) / ( dim_a*cos(delta)*cos(delta) );
    double time_diff = wheel_msg.z - time_prev;
    wheel_yaw += yaw_rate * time_diff;
    wheel_x += u*cos(wheel_yaw) * time_diff;
    wheel_y += u*sin(wheel_yaw) * time_diff; //account for REP-103 sign convention
    tf::Quaternion q;
    q.setRPY(0,0,wheel_yaw);

    ROS_INFO("imu_pitch: %lf, imu_roll: %lf, yaw: %lf, x: %lf, y: %lf", imu_pitch, imu_roll, wheel_yaw, wheel_x, wheel_y);

    //Publish odom message
    nav_msgs::Odometry odom; //instantiate odom message
    
    odom.pose.pose.position.x = wheel_x;
    odom.pose.pose.position.y = wheel_y;
    odom.pose.pose.position.z= 0.0;
    tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
    odom.pose.covariance[0] = 0.1; //x
    odom.pose.covariance[7] = 0.1; //y
    odom.pose.covariance[0] = 100; //z
    odom.pose.covariance[0] = 100; //roll
    odom.pose.covariance[0] = 100; //pitch
    odom.pose.covariance[0] = 0.1; //yaw
   
    odom.twist.twist.linear.x = u*cos(wheel_yaw);
    odom.twist.twist.linear.y = u*sin(wheel_yaw);
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0; //roll
    odom.twist.twist.angular.y = 0; //pitch
    odom.twist.twist.angular.z = yaw_rate; //yaw
    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[0] = 100;
    odom.twist.covariance[0] = 100;
    odom.twist.covariance[0] = 100;
    odom.twist.covariance[0] = 0.1;
   
    odom.child_frame_id = child_frame_id;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = parent_frame_id;

    odom_pub.publish(odom);

    //Broadcast odom transform
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = parent_frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = wheel_x;
    odom_trans.transform.translation.y = wheel_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom.pose.pose.orientation;
    odom_broadcaster->sendTransform(odom_trans);

  }
  time_prev = wheel_msg.z;
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "example_odom");
  ros::NodeHandle nh;
  std::string initial_pose, longboard_dim;
  //read initial pose parameter; if it doesn't exist, populate initial pose with default string of zeroes
  nh.param<std::string>("initial_pose", initial_pose, "0 0 0"); //string containing initial x y yaw of robot
  nh.param<std::string>("longboard_dim", longboard_dim,"0.872639 0.8382"); //string containing angle of trucks in radians (assumed equal for both trucks) and distance between axles in meters
  nh.param<std::string>("child_frame_id", child_frame_id,"base_link"); //child frame id string
  nh.param<std::string>("parent_frame_id", parent_frame_id,"odom"); //(presumably) parent frame id string

  //parse parameter string!
  sscanf(initial_pose.c_str(), "%lf %lf %lf", &wheel_x, &wheel_y, &wheel_yaw);
  sscanf(longboard_dim.c_str(), "%lf %lf", &dim_a, &dim_d);


  // Create a ROS subscriber for imu data
  ros::Subscriber imu_sub = nh.subscribe ("imu", 1, imu_cb);
  ros::Subscriber wheel_sub = nh.subscribe ("wheel", 1, wheel_cb);
  odom_pub = nh.advertise<nav_msgs::Odometry> ("wheel_odom", 1);
  odom_broadcaster = new tf::TransformBroadcaster();
  ROS_INFO("%s","Odom node established");
  // Spin
  ros::spin ();
}