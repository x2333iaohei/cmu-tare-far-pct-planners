#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
// #include "tf2_ros/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>   // 将tf监听绑定到ROS回调函数
#include <cmath>


using namespace std;
ros::Publisher robotVelocity_BASE_frame_pub;
string robot_name = "a1";
nav_msgs::Odometry Odom;
double x=0, y=0, z=0, roll=0, pitch=0, yaw=0;

static bool finiteDouble(double value) {
    return std::isfinite(value);
}

static bool finitePoint(const geometry_msgs::Point &point) {
    return finiteDouble(point.x) && finiteDouble(point.y) && finiteDouble(point.z);
}

static bool finiteQuaternion(const geometry_msgs::Quaternion &quat) {
    return finiteDouble(quat.x) && finiteDouble(quat.y) &&
           finiteDouble(quat.z) && finiteDouble(quat.w);
}

static bool finiteVector3(const geometry_msgs::Vector3 &vec) {
    return finiteDouble(vec.x) && finiteDouble(vec.y) && finiteDouble(vec.z);
}

static bool validQuaternion(const geometry_msgs::Quaternion &quat) {
    if (!finiteQuaternion(quat)) {
        return false;
    }
    const double norm_sq = quat.x * quat.x + quat.y * quat.y +
                           quat.z * quat.z + quat.w * quat.w;
    return finiteDouble(norm_sq) && norm_sq > 1e-8;
}

void callback_BASE(const gazebo_msgs::LinkStates::ConstPtr &msg) {
    int index = 0;
    for (auto &linkName : msg->name) {
        if (linkName == robot_name+"_gazebo::base")
            break;
        ++index;
    }
    if (index >= static_cast<int>(msg->name.size())) {
        ROS_WARN_THROTTLE(2.0, "Could not find %s_gazebo::base in /gazebo/link_states", robot_name.c_str());
        return;
    }

    const auto &pose = msg->pose[index];
    const auto &twist = msg->twist[index];
    if (!finitePoint(pose.position) || !validQuaternion(pose.orientation)) {
        ROS_WARN_THROTTLE(1.0, "Skipping invalid %s_gazebo::base pose from Gazebo", robot_name.c_str());
        return;
    }
    if (!finiteVector3(twist.linear) || !finiteVector3(twist.angular)) {
        ROS_WARN_THROTTLE(1.0, "Skipping invalid %s_gazebo::base twist from Gazebo", robot_name.c_str());
        return;
    }
    if (pose.position.z < 0.05) {
        ROS_WARN_THROTTLE(1.0, "Skipping implausible %s_gazebo::base pose z=%.3f from Gazebo", robot_name.c_str(), pose.position.z);
        return;
    }

    const ros::Time stamp = ros::Time::now();
    static ros::Time last_stamp;
    if (!stamp.isZero() && stamp <= last_stamp) {
        return;
    }
    last_stamp = stamp;

    //map到odom的tf变换
    static tf::TransformBroadcaster bf1;
    tf::Transform transform_odom2map;
    tf::Quaternion qtn;
    qtn.setRPY(roll, pitch, yaw);
    transform_odom2map.setRotation(tf::Quaternion(qtn.x(),
                                         qtn.y(),
                                         qtn.z(),
                                         qtn.w()));
    transform_odom2map.setOrigin(tf::Vector3(x,
                                    y,
                                    z));
    // 发布odom到map的tf关系
    bf1.sendTransform(tf::StampedTransform(transform_odom2map, stamp, "map", "odom"));
    
    //求变化矩阵的逆解，用于推算map到odom的关系，以便能得到base到map的关系，及
    tf::Transform transform_map2odom = transform_odom2map.inverse();

    tf::Point pt_map(pose.position.x, pose.position.y, pose.position.z);
    tf::Point pt_odom = transform_map2odom * pt_map;
    
    tf::Quaternion q_map(pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w);
    tf::Quaternion q_odom = transform_map2odom.getRotation() * q_map;

    // 转换为odom的速度关系
    tf::Vector3 linear_vel(
        twist.linear.x,
        twist.linear.y,
        twist.linear.z);
    tf::Vector3 transformed_linear_vel = transform_map2odom * linear_vel;

    tf::Vector3 angular_vel(
        twist.angular.x,
        twist.angular.y,
        twist.angular.z);
    tf::Vector3 transformed_angular_vel = transform_map2odom * angular_vel;
    
    //发布base到odom的tf变换
    static tf::TransformBroadcaster bf2;
    tf::Transform transform_odom2base;
    transform_odom2base.setRotation(q_odom);
    transform_odom2base.setOrigin(pt_odom);

    bf2.sendTransform(tf::StampedTransform(transform_odom2base, stamp, "odom", "base"));

    Odom.header.stamp = stamp;
    Odom.header.frame_id = "odom";
    Odom.child_frame_id = "base";

    // set the position
    Odom.pose.pose.position.x = pt_odom.x();
    Odom.pose.pose.position.y = pt_odom.y();
    Odom.pose.pose.position.z = pt_odom.z();

    Odom.pose.pose.orientation.w = q_odom.w();
    Odom.pose.pose.orientation.x = q_odom.x();
    Odom.pose.pose.orientation.y = q_odom.y();
    Odom.pose.pose.orientation.z = q_odom.z();


    // set the velocity
    Odom.twist.twist.linear.x= transformed_linear_vel.x();
    Odom.twist.twist.linear.y= transformed_linear_vel.y();
    Odom.twist.twist.linear.z= transformed_linear_vel.z();


    Odom.twist.twist.angular.x = transformed_angular_vel.x();
    Odom.twist.twist.angular.y = transformed_angular_vel.y();
    Odom.twist.twist.angular.z = transformed_angular_vel.z();


    robotVelocity_BASE_frame_pub.publish(Odom);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "state_from_gazebo");
    ros::NodeHandle nh("~");
    ros::NodeHandle node;
    ros::Subscriber tfState_BASE_sub;

    // tf::TransformListener tf_listener_;

    if (argc != 7)   // x y z qx qy qz qw 
    {
        ROS_ERROR("Usage: static_transform_publisher x y z yaw pitch roll");
        return -1;
    }

    x = atof(argv[1]);
    y = atof(argv[2]);
    z = atof(argv[3]);

    yaw   = atof(argv[4]);
    pitch = atof(argv[5]);
    roll  = atof(argv[6]);
  
    nh.param<std::string>("robot_name", robot_name, string("a1"));
    tfState_BASE_sub = node.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, callback_BASE);
    robotVelocity_BASE_frame_pub = node.advertise<nav_msgs::Odometry>("/Odometry_gazebo", 1);

    ros::spin();
    return 0;
}

