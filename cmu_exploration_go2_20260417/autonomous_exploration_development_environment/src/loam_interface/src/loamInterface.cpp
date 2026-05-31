#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

// 参数：状态估计（里程计）的输入话题名称
string stateEstimationTopic = "/integrated_to_init";
// 参数：已配准的点云输入话题名称
string registeredScanTopic = "/velodyne_cloud_registered";
// 参数：是否对状态估计数据做坐标翻转（交换并取反某些轴）
bool flipStateEstimation = true;
// 参数：是否对已配准点云做坐标翻转（交换坐标轴）
bool flipRegisteredScan = true;
// 参数：是否发布TF变换
bool sendTF = true;
// 参数：是否反转TF变换（即 sensor -> map 而非 map -> sensor）
bool reverseTF = false;

// 存储点云数据的PCL点云对象
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

// 存储最新的里程计数据
nav_msgs::Odometry odomData;
// 存储里程计对应的TF变换
tf::StampedTransform odomTrans;
// 指向里程计发布者的指针（用于在回调中发布）
ros::Publisher *pubOdometryPointer = NULL;
// 指向TF广播器的指针
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
// 指向点云发布者的指针
ros::Publisher *pubLaserCloudPointer = NULL;

// 里程计话题的回调函数
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  odomData = *odom;

  // 如果需要翻转状态估计（坐标轴变换）
  if (flipStateEstimation) {
    // 将四元数转换为欧拉角，注意这里交换了四元数的x,y,z符号实现坐标轴重映射
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    pitch = -pitch;
    yaw = -yaw;

    // 根据翻转后的欧拉角重新生成四元数
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    // 对位置坐标进行交换：原z->新x，原x->新y，原y->新z
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = odom->pose.pose.position.z;
    odomData.pose.pose.position.y = odom->pose.pose.position.x;
    odomData.pose.pose.position.z = odom->pose.pose.position.y;
  }

  // 发布转换后的里程计消息（frame_id固定为"map"，child_frame_id为"sensor"）
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";
  pubOdometryPointer->publish(odomData);

  // 根据里程计数据构建TF变换（map -> sensor）
  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "sensor";
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

  // 如果需要发送TF
  if (sendTF) {
    if (!reverseTF) {
      // 正常发送 map -> sensor 的变换
      tfBroadcasterPointer->sendTransform(odomTrans);
    } else {
      // 反转TF，发送 sensor -> map 的变换
      tfBroadcasterPointer->sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "sensor", "map"));
    }
  }
}

// 已配准点云话题的回调函数
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  laserCloud->clear();
  // 将ROS点云消息转换为PCL点云
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  // 如果需要翻转点云坐标（交换x,y,z轴）
  if (flipRegisteredScan) {
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      // 坐标变换：原(x,y,z) -> (z, x, y) 即交换并循环移位
      float temp = laserCloud->points[i].x;
      laserCloud->points[i].x = laserCloud->points[i].z;
      laserCloud->points[i].z = laserCloud->points[i].y;
      laserCloud->points[i].y = temp;
    }
  }

  // 将处理后的PCL点云转换回ROS消息并发布（frame_id设置为"map"）
  sensor_msgs::PointCloud2 laserCloud2;
  pcl::toROSMsg(*laserCloud, laserCloud2);
  laserCloud2.header.stamp = laserCloudIn->header.stamp;
  laserCloud2.header.frame_id = "map";
  pubLaserCloudPointer->publish(laserCloud2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  // 从参数服务器读取配置参数
  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);

  // 订阅输入里程计话题
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, odometryHandler);

  // 订阅输入已配准点云话题
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (registeredScanTopic, 5, laserCloudHandler);

  // 发布转换后的里程计话题
  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
  pubOdometryPointer = &pubOdometry;

  // 实例化TF广播器
  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  // 发布转换后的点云话题
  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);
  pubLaserCloudPointer = &pubLaserCloud;

  // 进入ROS事件循环
  ros::spin();

  return 0;
}