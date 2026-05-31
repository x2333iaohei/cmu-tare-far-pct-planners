#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

// ======================== 参数配置（可通过 launch 文件或参数服务器修改） ========================
double sensorOffsetX = 0;          // 传感器相对于车辆中心的X偏移（米）
double sensorOffsetY = 0;          // 传感器相对于车辆中心的Y偏移（米）
int pubSkipNum = 1;                // 控制指令发布跳帧数（每N帧发布一次）
int pubSkipCount = 0;              // 跳帧计数器
bool twoWayDrive = true;           // 是否允许双向行驶（倒车）
double lookAheadDis = 0.8;         // 路径跟踪的预瞄距离（米）【建议0.8】
double yawRateGain = 1.2;          // 正常行驶时的偏航率控制增益【降低】
double stopYawRateGain = 1.0;      // 低速（接近停止）时的偏航率控制增益【降低】
double maxYawRate = 25.0;          // 最大偏航率（度/秒）【降低至25】
double maxSpeed = 1.0;             // 最大线速度（米/秒）
double maxAccel = 0.8;             // 最大加速度（米/秒²）【降低】
double switchTimeThre = 1.0;       // 前进/后退方向切换的时间阈值（秒）
double dirDiffThre = 0.3;          // 方向误差阈值（弧度），用于判断是否对齐路径
double stopDisThre = 0.2;          // 停止距离阈值（米），小于此距离认为已到达目标点
double slowDwnDisThre = 1.0;       // 减速起始距离阈值（米）
bool useInclRateToSlow = false;    // 是否使用倾斜角速度来触发减速
double inclRateThre = 120.0;       // 倾斜角速度阈值（度/秒），超过则减速
double slowRate1 = 0.25;           // 第一段减速的速度系数
double slowRate2 = 0.5;            // 第二段减速的速度系数
double slowTime1 = 2.0;            // 第一段减速持续时间（秒）
double slowTime2 = 2.0;            // 第二段减速持续时间（秒）
bool useInclToStop = false;        // 是否使用倾斜角度来触发停车
double inclThre = 45.0;            // 倾斜角度阈值（度），超过则停车
double stopTime = 5.0;             // 停车持续时间（秒）
bool noRotAtStop = false;          // 停车时是否禁止转向
bool noRotAtGoal = true;           // 到达目标点时是否禁止转向
bool autonomyMode = false;         // 是否自主模式（true时忽略摇杆速度，使用外部速度指令）
double autonomySpeed = 1.0;        // 自主模式下的固定速度（米/秒）
double joyToSpeedDelay = 2.0;      // 摇杆无输入后切换到外部速度指令的延迟（秒）

// 摇杆输入处理后的速度和方向（归一化）
float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;                  // 摇杆转向指令（-1..1）
int safetyStop = 0;               // 外部紧急停车标志（0=正常，1=仅停车，2=停车+禁止转向）

// 车辆当前状态（位置、姿态）
float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

// 记录接收到路径时的车辆状态（用于路径坐标系转换）
float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

// 控制量
float vehicleYawRate = 0;          // 目标偏航率（rad/s）
float vehicleSpeed = 0;            // 目标线速度（m/s）

// 时间戳
double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;           // 触发减速的开始时间
double stopInitTime = false;       // 触发停车的开始时间（实际当作double用，但初始为false）
int pathPointID = 0;               // 当前跟踪的路径点索引
bool pathInit = false;             // 是否已接收到有效路径
bool navFwd = true;                // 当前行驶方向（true=前进，false=后退）
double switchTime = 0;             // 方向切换时间记录

nav_msgs::Path path;               // 存储当前全局路径

// ======================== 回调函数 ========================

// 里程计回调：更新车辆位姿，计算倾斜触发停车/减速条件
void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  // 根据传感器安装偏移计算车辆中心位置
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  // 检查倾斜角度是否超过阈值（停车条件）
  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  // 检查倾斜角速度是否超过阈值（减速条件）
  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = odomIn->header.stamp.toSec();
  }
}

// 路径回调：接收局部规划器发布的路径，并记录接收时的车辆位姿
void pathHandler(const nav_msgs::Path::ConstPtr& pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  // 记录接收到路径时的车辆状态（用于将路径点转换到车辆初始坐标系）
  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

// 摇杆回调：获取手动控制指令，切换自主模式
void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyTime = ros::Time::now().toSec();

  // 计算摇杆速度（二维矢量模长）
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  // 如果不允许双向行驶，倒车指令无效
  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  // 按钮（axes[2]）控制自主模式：按下（值<= -0.1）进入自主模式
  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
  autonomyMode = true; // cjw tmp  (强制自主模式，调试用)
}

// 外部速度指令回调（当摇杆无输入超时后，使用此速度）
void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;
    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

// 紧急停车标志回调
void stopHandler(const std_msgs::Int8::ConstPtr& stop)
{
  safetyStop = stop->data;
}

// ======================== 主函数 ========================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  // 从参数服务器加载配置
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("lookAheadDis", lookAheadDis);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("switchTimeThre", switchTimeThre);
  nhPrivate.getParam("dirDiffThre", dirDiffThre);
  nhPrivate.getParam("stopDisThre", stopDisThre);
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
  nhPrivate.getParam("inclRateThre", inclRateThre);
  nhPrivate.getParam("slowRate1", slowRate1);
  nhPrivate.getParam("slowRate2", slowRate2);
  nhPrivate.getParam("slowTime1", slowTime1);
  nhPrivate.getParam("slowTime2", slowTime2);
  nhPrivate.getParam("useInclToStop", useInclToStop);
  nhPrivate.getParam("inclThre", inclThre);
  nhPrivate.getParam("stopTime", stopTime);
  nhPrivate.getParam("noRotAtStop", noRotAtStop);
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);

  // 订阅话题
  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odomHandler);
  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> ("/path", 5, pathHandler);
  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);
  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);
  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8> ("/stop", 5, stopHandler);

  // 发布控制指令
  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped> ("/cmd_vel", 5);
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";

  // 若自主模式，初始速度使用 autonomySpeed
  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;
    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (pathInit) {
      // 将当前车辆位置转换到路径记录时的车辆坐标系（路径点在该坐标系下定义）
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      // 计算路径终点与当前车辆的相对距离
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      // 预瞄点搜索：找到第一个距离大于 lookAheadDis 的路径点
      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      // 计算预瞄点相对于车辆的方向角
      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      // 计算方向误差（考虑车辆当前朝向与路径记录时的朝向差）
      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      // 双向行驶时，根据方向误差自动决定前进/后退
      if (twoWayDrive) {
        double time = ros::Time::now().toSec();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;   // 方向误差过大且当前为前进，切换为后退
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;    // 方向误差变小且当前为后退，切换为前进
          switchTime = time;
        }
      }

      // 计算期望速度（考虑方向）
      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;   // 后退时速度为负
      }

      // ========== 1. 原始偏航率计算 ==========
      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      // 限制最大偏航率（硬限幅）
      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      // ========== 2. 角速度低通滤波（平滑转向，防止突变）==========
      static float lastYawRate = 0.0;
      float yawRateLPF = 0.3;   // 滤波系数，越小越平滑（0.2~0.4）
      vehicleYawRate = yawRateLPF * vehicleYawRate + (1.0 - yawRateLPF) * lastYawRate;
      lastYawRate = vehicleYawRate;

      // ========== 3. 角加速度限制（防止急转） ==========
      static float lastYawRateCmd = 0.0;
      float maxYawAccel = 60.0 * PI / 180.0;   // 最大角加速度 60°/s²
      float deltaYawRate = vehicleYawRate - lastYawRateCmd;
      float maxDelta = maxYawAccel / 100.0;    // 每控制周期允许的最大变化（100Hz）
      if (deltaYawRate > maxDelta)
        vehicleYawRate = lastYawRateCmd + maxDelta;
      else if (deltaYawRate < -maxDelta)
        vehicleYawRate = lastYawRateCmd - maxDelta;
      lastYawRateCmd = vehicleYawRate;

      // 非自主模式下且速度为0时，允许摇杆直接控制转向（不加平滑）
      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } 
      // 路径点数不足1个，或已到达终点且禁止转向时，偏航率为0
      else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      // 如果没有有效路径，速度设为0
      if (pathSize <= 1) {
        joySpeed2 = 0;
      } 
      // 根据距终点的距离进行线性减速
      else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      joySpeed3 = joySpeed3 > maxSpeed ? maxSpeed : joySpeed3;
      joySpeed3 = joySpeed3 < -maxSpeed ? -maxSpeed : joySpeed3;

      // ========== 4. 过弯自动减速（根据角速度大小降低线速度） ==========
      float absYawRate = fabs(vehicleYawRate);
      float maxAllowedYaw = maxYawRate * PI / 180.0;
      float speedScale = 1.0 - (absYawRate / maxAllowedYaw) * 0.6;   // 最大降速60%
      if (speedScale < 0.4) speedScale = 0.4;
      joySpeed3 *= speedScale;

      // 倾斜角速度触发的减速（分级减速）
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;

      // 加速度控制：根据方向误差和距离决定是否加速
      if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }

      // 倾斜角度触发的紧急停车
      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      // 外部安全停车标志处理
      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      // 发布控制指令（跳帧发布）
      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ros::Time().fromSec(odomTime);
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.twist.linear.x = 0;
        else cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        pubSpeed.publish(cmd_vel);

        pubSkipCount = pubSkipNum;
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}