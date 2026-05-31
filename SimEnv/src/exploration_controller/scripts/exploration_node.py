#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
探索节点 (Exploration Node)
实现自主探索策略，控制机器人沿墙行走探索未知环境

功能：
1. 订阅点云数据，检测障碍物距离
2. 订阅里程计，获取机器人当前位置和朝向
3. 实现沿墙行走 + 边界探索策略
4. 发布目标点供控制节点使用
5. 自动开关门和电梯控制
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from building_generator_interfaces.srv import SetDoorState, CallElevator

class ExplorationNode:
    """
    探索节点类
    采用沿墙行走策略实现自主探索
    自动处理门和电梯控制
    """

    def __init__(self):
        # 加载参数
        self.min_range = rospy.get_param('~exploration/min_range', 0.5)
        self.max_range = rospy.get_param('~exploration/max_range', 3.0)
        self.safe_range = rospy.get_param('~exploration/safe_range', 1.5)
        self.turn_speed = rospy.get_param('~exploration/turn_speed', 0.5)
        self.forward_speed = rospy.get_param('~exploration/forward_speed', 0.3)
        self.backward_speed = rospy.get_param('~exploration/backward_speed', 0.15)
        self.wall_follow_dist = rospy.get_param('~exploration/wall_follow_dist', 1.0)

        # 机器人状态
        self.current_pose = None  # 当前位姿 [x, y, theta]
        self.current_z = 0.0  # 用于检测楼层变化
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 订阅话题
        rospy.Subscriber('/Odometry_gazebo', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', PointCloud2, self.scan_callback)

        # 门/电梯服务客户端
        self.door_service = rospy.ServiceProxy('/set_door_state', SetDoorState)
        self.elevator_service = rospy.ServiceProxy('/call_elevator', CallElevator)

        # 等待服务就绪
        rospy.loginfo("[探索] 等待门/电梯控制服务...")
        rospy.wait_for_service('/set_door_state', timeout=5.0)
        rospy.loginfo("[探索] 门/电梯服务已就绪")

        # 探索状态机
        self.state = 'EXPLORE'  # EXPLORE: 探索模式, TURN_LEFT: 左转, TURN_RIGHT: 右转, BACKWARD: 后退
        self.turn_start_time = None
        self.turn_duration = 0.5  # 旋转持续时间(秒)
        self.explore_done = False  # 是否已完成探索
        self.stuck_counter = 0  # 卡住计数器
        self.last_front_distance = float('inf')
        self.current_floor = 0  # 当前楼层
        self.forward_counter = 0  # 前进计数器（用于决定何时随机转向）
        self.random_turn_interval = 20  # 每多少次循环随机转向一次

        # 激光数据（简化为距离值）
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.front_left_distance = float('inf')
        self.front_right_distance = float('inf')

        # 初始化时打开主入口门
        self.open_door('main_entrance')

        rospy.loginfo("[探索节点] 初始化完成")

    def open_door(self, door_id):
        """打开指定的门"""
        try:
            resp = self.door_service(door_id=door_id, open=True)
            rospy.loginfo("[探索] 打开门: %s", door_id)
        except rospy.ServiceException as e:
            rospy.logerr("[探索] 门服务调用失败: %s", str(e))

    def close_door(self, door_id):
        """关闭指定的门"""
        try:
            resp = self.door_service(door_id=door_id, open=False)
            rospy.loginfo("[探索] 关闭门: %s", door_id)
        except rospy.ServiceException as e:
            rospy.logerr("[探索] 门服务调用失败: %s", str(e))

    def call_elevator(self, target_floor, open_doors=False):
        """调用电梯到目标楼层"""
        try:
            resp = self.elevator_service(elevator_id='elevator_main', target_floor=target_floor, open_doors=open_doors)
            rospy.loginfo("[探索] 电梯前往 %d 楼", target_floor)
        except rospy.ServiceException as e:
            rospy.logerr("[探索] 电梯服务调用失败: %s", str(e))

    def odom_callback(self, msg):
        """里程计回调，更新机器人当前位置"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # 四元数转欧拉角
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = [pos.x, pos.y, yaw]
        self.current_z = pos.z

        # 根据z坐标判断当前楼层
        if pos.z < 0.5:
            self.current_floor = 0
        elif pos.z < 3.0:
            self.current_floor = 1
        else:
            self.current_floor = 2

    def scan_callback(self, msg):
        """
        点云回调，提取各方向障碍物距离
        将360度点云简化为5个方向的距离
        """
        points = []
        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])
        except:
            return

        if not points:
            return

        # 前方, 右前45度, 右90度, 左前-45度, 左-90度
        angles = [0, math.pi/4, math.pi/2, -math.pi/4, -math.pi/2]
        distances = [float('inf')] * 5

        for i, angle in enumerate(angles):
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            for pt in points:
                proj = pt[0] * cos_a + pt[1] * sin_a
                if proj > 0:
                    lateral = abs(pt[0] * sin_a - pt[1] * cos_a)
                    if lateral < 0.5:
                        dist = math.sqrt(pt[0]**2 + pt[1]**2)
                        if dist < distances[i]:
                            distances[i] = dist

        self.front_distance = distances[0]
        self.front_right_distance = distances[1]
        self.right_distance = distances[2]
        self.front_left_distance = distances[3]
        self.left_distance = distances[4]

    def compute_cmd_vel(self):
        """计算速度指令 - 改进的探索策略"""
        cmd = Twist()

        # 状态机逻辑
        if self.state == 'EXPLORE':
            # 探索模式：保持与墙壁的安全距离，避免贴墙
            # 如果右距离太小或太大，调整角度
            if self.right_distance < self.wall_follow_dist * 0.6:
                # 右距离太小，左转远离墙壁
                cmd.linear.x = self.forward_speed * 0.5
                cmd.angular.z = self.turn_speed * 0.5
            elif self.right_distance > self.wall_follow_dist * 2.0:
                # 右距离太大，右转靠近墙壁
                cmd.linear.x = self.forward_speed * 0.8
                cmd.angular.z = -self.turn_speed * 0.3
            elif self.front_distance < self.min_range * 1.5:
                # 前方距离变近，右转
                self.state = 'TURN_RIGHT'
                self.turn_start_time = rospy.Time.now()
                rospy.loginfo("[探索] 前方变窄，开始右转")
            else:
                # 正常前进，稍微右转保持沿墙
                cmd.linear.x = self.forward_speed
                cmd.angular.z = -0.1  # 稍微右转，沿墙走

            # 计数器用于随机转向
            self.forward_counter += 1
            if self.forward_counter > self.random_turn_interval:
                self.forward_counter = 0
                # 随机决定左转还是右转
                import random
                if random.random() < 0.5:
                    self.state = 'TURN_LEFT'
                    self.turn_start_time = rospy.Time.now()
                    rospy.loginfo("[探索] 随机左转探索新区域")
                else:
                    self.state = 'TURN_RIGHT'
                    self.turn_start_time = rospy.Time.now()
                    rospy.loginfo("[探索] 随机右转探索新区域")

        elif self.state == 'FORWARD':
            # 前方有障碍物
            if self.front_distance < self.min_range:
                self.state = 'TURN_RIGHT'
                self.turn_start_time = rospy.Time.now()
                rospy.loginfo("[探索] 前方障碍物，开始右转")
            elif self.right_distance > self.wall_follow_dist * 1.5:
                self.state = 'TURN_LEFT'
                self.turn_start_time = rospy.Time.now()
                rospy.loginfo("[探索] 右距离变大，开始左转探索")
            elif self.right_distance < self.min_range:
                self.state = 'TURN_LEFT'
                self.turn_start_time = rospy.Time.now()
                rospy.loginfo("[探索] 右距离太近，左转避让")
            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
                self.stuck_counter = 0
            # 前方有障碍物
            if self.front_distance < self.min_range:
                self.state = 'TURN_RIGHT'
                self.turn_start_time = rospy.Time.now()
                rospy.loginfo("[探索] 前方障碍物，开始右转")
            elif self.right_distance > self.wall_follow_dist * 1.5:
                self.state = 'TURN_LEFT'
                self.turn_start_time = rospy.Time.now()
                rospy.loginfo("[探索] 右距离变大，开始左转探索")
            elif self.right_distance < self.min_range:
                self.state = 'TURN_LEFT'
                self.turn_start_time = rospy.Time.now()
                rospy.loginfo("[探索] 右距离太近，左转避让")
            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
                self.stuck_counter = 0

        elif self.state == 'TURN_RIGHT':
            cmd.linear.x = 0.0
            cmd.angular.z = -self.turn_speed
            if (rospy.Time.now() - self.turn_start_time).to_sec() > self.turn_duration:
                self.state = 'FORWARD'

        elif self.state == 'TURN_LEFT':
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
            if (rospy.Time.now() - self.turn_start_time).to_sec() > self.turn_duration:
                self.state = 'FORWARD'

        elif self.state == 'BACKWARD':
            cmd.linear.x = -self.forward_speed
            cmd.angular.z = 0.0
            if (rospy.Time.now() - self.turn_start_time).to_sec() > 0.5:
                self.state = 'TURN_LEFT'
                self.turn_start_time = rospy.Time.now()
                self.stuck_counter = 0
                rospy.loginfo("[探索] 后退后尝试左转")

        # 卡住检测
        if self.state in ['TURN_RIGHT', 'TURN_LEFT']:
            if abs(self.front_distance - self.last_front_distance) < 0.1:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0

            if self.stuck_counter > 10:
                rospy.logwarn("[探索] 检测到卡住，后退并尝试其他方向")
                self.state = 'BACKWARD'
                self.turn_start_time = rospy.Time.now()

        self.last_front_distance = self.front_distance
        return cmd

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz控制频率
        while not rospy.is_shutdown():
            if self.current_pose is not None:
                cmd = self.compute_cmd_vel()
                self.cmd_vel_pub.publish(cmd)

                if int(rospy.Time.now().to_sec()) % 5 == 0:
                    rospy.loginfo("[探索] 状态:%s, 前:%.2f, 右:%.2f, 楼层:%d" % (
                        self.state, self.front_distance, self.right_distance, self.current_floor))
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('exploration_node', anonymous=True)
    try:
        node = ExplorationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass