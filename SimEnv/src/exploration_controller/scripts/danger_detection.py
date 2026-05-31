#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
危险源检测节点 (Danger Detection Node)
检测环境中的红色球体（危险源）

规则验证：
1. 只在layout_metadata.json登记的房间边界内部采样
2. 距房间墙体保持安全边距（0.3m）
3. 距家具保持安全边距，避免与桌椅、柜体等重叠
4. 多个源之间保持最小间距（0.5m）
5. 房门附近设置保留区，不在门口及短通道上放置
6. 球体中心高度 = 楼层高度 + 半径（0.15m）
7. 危险源：红色球体；干扰源：绿色球体、红色方块
"""

import rospy
import cv2
import numpy as np
import json
import os
import math
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

class DangerDetectionNode:
    """
    危险源检测节点类
    检测红色球体（危险源），排除干扰源（绿色球体、红色方块）
    """

    def __init__(self):
        # 加载参数
        self.red_hsv_low_h = rospy.get_param('~detection/red_hsv_low_h', 0)
        self.red_hsv_low_s = rospy.get_param('~detection/red_hsv_low_s', 120)
        self.red_hsv_low_v = rospy.get_param('~detection/red_hsv_low_v', 100)
        self.red_hsv_high_h = rospy.get_param('~detection/red_hsv_high_h', 10)
        self.red_hsv_high_s = rospy.get_param('~detection/red_hsv_high_s', 255)
        self.red_hsv_high_v = rospy.get_param('~detection/red_hsv_high_v', 255)
        self.min_ball_radius = rospy.get_param('~detection/min_ball_radius', 10)
        self.max_ball_radius = rospy.get_param('~detection/max_ball_radius', 100)
        self.depth_filter_min = rospy.get_param('~detection/depth_filter_min', 0.3)
        self.depth_filter_max = rospy.get_param('~detection/depth_filter_max', 5.0)

        self.results_dir = rospy.get_param('~output/results_dir', '/home/xiaohei/SimEnv/results')
        self.output_interval = rospy.get_param('~output/output_interval', 10.0)

        # 加载环境数据
        self.room_data = self.load_environment_data()

        # 初始化CV Bridge
        self.bridge = CvBridge()

        # 当前数据
        self.current_image = None
        self.current_depth_points = None
        self.current_odom = None

        # 检测确认机制
        self.pending_detections = {}
        self.confirmation_threshold = 3
        self.confirmation_radius = 0.5  # 0.5m最小间距
        self.confirmation_valid_seconds = 5.0

        # 已确认的危险源
        self.confirmed_dangers = []

        # 探索开始时间
        self.exploration_start_time = rospy.Time.now()

        # 订阅话题
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/real_sense/depth/points', PointCloud2, self.depth_callback)
        rospy.Subscriber('/Odometry_gazebo', Odometry, self.odom_callback)

        # 定时输出结果
        rospy.Timer(rospy.Duration(self.output_interval), self.timer_callback)

        rospy.loginfo("[危险源检测] 初始化完成")
        rospy.loginfo("[危险源检测] 房间数:%d, 门数:%d, 家具数:%d",
                     len(self.room_data['rooms']),
                     len(self.room_data['doors']),
                     len(self.room_data['furniture']))

    def load_environment_data(self):
        """加载房间、门、家具数据"""
        data = {'rooms': [], 'doors': [], 'furniture': []}
        layout_file = '/home/xiaohei/SimEnv/generated_building/layout_metadata.json'

        try:
            with open(layout_file, 'r') as f:
                layout = json.load(f)

            # 加载楼层高度
            floor_height = layout.get('floor_height', 2.6)
            floor_heights = layout.get('building', {}).get('floor_heights', [0.0, 2.6, 5.2])

            for floor in layout.get('floors', []):
                floor_index = floor.get('floor_index', 0)
                elevation = floor.get('elevation', 0)

                # 加载房间边界
                for room in floor.get('rooms', []):
                    bounds = room.get('bounds', {})
                    room_id = room.get('id', '')

                    # 房间边界（含安全边距0.3m）
                    wall_margin = 0.3
                    data['rooms'].append({
                        'room_id': room_id,
                        'floor_index': floor_index,
                        'elevation': elevation,
                        'x_min': bounds.get('x_min', 0) + wall_margin,
                        'x_max': bounds.get('x_max', 0) - wall_margin,
                        'y_min': bounds.get('y_min', 0) + wall_margin,
                        'y_max': bounds.get('y_max', 0) - wall_margin,
                        # 球体高度验证：楼层高度+0.15
                        'z_expected': floor_heights[floor_index] + 0.15
                    })

                # 加载门位置（门口保留区）
                for door in layout.get('door_specs', []):
                    if door.get('floor_index') == floor_index:
                        pose = door.get('pose', [])
                        data['doors'].append({
                            'door_id': door.get('id', ''),
                            'floor_index': floor_index,
                            'x': pose[0] if len(pose) > 0 else 0,
                            'y': pose[1] if len(pose) > 1 else 0,
                            'z': pose[2] if len(pose) > 2 else elevation,
                            # 门口保留区半径1.5m
                            'keepout_radius': 1.5
                        })

                # 加载家具边界
                for room in floor.get('rooms', []):
                    for furn in room.get('furniture', []):
                        pose = furn.get('pose', [])
                        size = furn.get('size', [])
                        if len(pose) >= 3 and len(size) >= 3:
                            furn_x = pose[0]
                            furn_y = pose[1]
                            furn_z = pose[2]
                            # 家具安全边距0.35m
                            furn_margin = 0.35
                            data['furniture'].append({
                                'id': furn.get('id', ''),
                                'floor_index': floor_index,
                                'x_min': furn_x - size[0]/2 - furn_margin,
                                'x_max': furn_x + size[0]/2 + furn_margin,
                                'y_min': furn_y - size[1]/2 - furn_margin,
                                'y_max': furn_y + size[1]/2 + furn_margin,
                                'z': furn_z
                            })

            rospy.loginfo("[危险源检测] 环境数据加载完成: %d房间, %d门, %d家具",
                        len(data['rooms']), len(data['doors']), len(data['furniture']))

        except Exception as e:
            rospy.logerr("[危险源检测] 加载环境数据失败: %s", str(e))

        return data

    def is_valid_danger_position(self, x, y, z):
        """
        验证检测位置是否有效
        1. 是否在房间内
        2. 是否在门口保留区外
        3. 是否在家具安全边距外
        4. 高度是否符合球体中心高度
        """
        for room in self.room_data['rooms']:
            # 检查楼层
            if abs(z - room['elevation']) > 0.5:
                continue

            # 检查是否在房间边界内
            if not (room['x_min'] <= x <= room['x_max'] and
                    room['y_min'] <= y <= room['y_max']):
                continue

            # 检查高度是否符合预期（球体中心高度 = 楼层高度 + 0.15）
            expected_z = room['z_expected']
            if abs(z - expected_z) > 0.2:
                return False, room['room_id'], "高度不符"

            # 检查是否在门口保留区
            for door in self.room_data['doors']:
                if door['floor_index'] != room['floor_index']:
                    continue
                dist = math.sqrt((x - door['x'])**2 + (y - door['y'])**2)
                if dist < door['keepout_radius']:
                    return False, room['room_id'], "门口保留区"

            # 检查是否在家具安全边距外
            for furn in self.room_data['furniture']:
                if furn['floor_index'] != room['floor_index']:
                    continue
                if (furn['x_min'] <= x <= furn['x_max'] and
                    furn['y_min'] <= y <= furn['y_max']):
                    return False, room['room_id'], "家具附近"

            # 检查与已确认危险源的最小间距
            for confirmed in self.confirmed_dangers:
                dist = math.sqrt((x - confirmed[0])**2 + (y - confirmed[1])**2)
                if dist < 0.5:
                    return False, room['room_id'], "间距不足"

            return True, room['room_id'], "有效"

        return False, None, "不在房间内"

    def odom_callback(self, msg):
        """里程计回调"""
        self.current_odom = msg

    def image_callback(self, msg):
        """图像回调"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("[危险源检测] CV Bridge错误: %s", str(e))
            return

        self.detect_red_balls()

    def depth_callback(self, msg):
        """深度点云回调"""
        self.current_depth_points = msg

    def detect_red_balls(self):
        """
        检测红色球体（危险源）
        排除干扰源：绿色球体、红色方块
        """
        if self.current_image is None:
            return

        hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)

        # 红色掩膜（两个范围）
        lower_red1 = np.array([0, self.red_hsv_low_s, self.red_hsv_low_v])
        upper_red1 = np.array([self.red_hsv_high_h, 255, 255])
        lower_red2 = np.array([170, self.red_hsv_low_s, self.red_hsv_low_v])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 形态学操作
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)

            # 过滤大小
            if radius < self.min_ball_radius or radius > self.max_ball_radius:
                continue

            # 计算圆形度 - 排除方块
            area = cv2.contourArea(contour)
            if area < 50:
                continue

            circularity = 4 * np.pi * area / (cv2.arcLength(contour, True) ** 2) if cv2.arcLength(contour, True) > 0 else 0
            # 球体圆形度应 > 0.7，方块会低很多
            if circularity < 0.7:
                rospy.logdebug("[危险源检测] 排除非圆形物体, 圆形度=%.2f", circularity)
                continue

            # 获取3D位置
            world_pos = self.get_3d_position(int(x), int(y), radius)

            if world_pos is not None:
                # 验证位置有效性
                valid, room_id, reason = self.is_valid_danger_position(
                    world_pos[0], world_pos[1], world_pos[2]
                )
                if valid:
                    self.add_pending_detection(world_pos, room_id)
                else:
                    rospy.logdebug("[危险源检测] 排除无效位置: [%s] %s", reason, world_pos)

    def get_3d_position(self, px, py, radius):
        """从深度点云获取3D位置"""
        if self.current_depth_points is None or self.current_odom is None:
            return None

        robot_pos = self.current_odom.pose.pose.position
        robot_ori = self.current_odom.pose.pose.orientation

        siny_cosp = 2 * (robot_ori.w * robot_ori.z + robot_ori.x * robot_ori.y)
        cosy_cosp = 1 - 2 * (robot_ori.y * robot_ori.y + robot_ori.z * robot_ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        fov = 1.396
        focal_length = 800 / (2 * np.tan(fov / 2))

        min_dist = float('inf')
        best_x, best_y, best_z = 0, 0, 0
        count = 0

        try:
            for p in pc2.read_points(self.current_depth_points,
                                      field_names=("x", "y", "z"),
                                      skip_nans=True):
                cam_x, cam_y, cam_z = p[0], p[1], p[2]
                dist = np.sqrt(cam_x**2 + cam_y**2 + cam_z**2)

                if dist < self.depth_filter_min or dist > self.depth_filter_max:
                    continue

                if abs(cam_z) < 0.01:
                    continue

                u = int(focal_length * cam_x / cam_z + 400)
                v = int(focal_length * cam_y / cam_z + 400)

                if abs(u - px) < radius * 2 and abs(v - py) < radius * 2:
                    if dist < min_dist:
                        min_dist = dist
                    best_x += cam_x
                    best_y += cam_y
                    best_z += cam_z
                    count += 1
        except:
            return None

        if count == 0:
            return None

        best_x /= count
        best_y /= count
        best_z /= count

        # 转换到世界坐标系
        world_x = robot_pos.x + best_z * np.cos(yaw) - best_x * np.sin(yaw)
        world_y = robot_pos.y + best_z * np.sin(yaw) + best_x * np.cos(yaw)
        world_z = robot_pos.z + best_y + 0.043

        return [world_x, world_y, world_z]

    def add_pending_detection(self, world_pos, room_id):
        """添加待确认的检测"""
        now = rospy.Time.now().to_sec()

        # 检查是否已确认
        for confirmed in self.confirmed_dangers:
            if np.linalg.norm(np.array(world_pos) - np.array(confirmed)) < self.confirmation_radius * 0.5:
                return

        # 查找匹配的位置
        matched_key = None
        for key, data in self.pending_detections.items():
            pos = key.split('_')
            pos = [float(pos[0]), float(pos[1]), float(pos[2])]
            if np.linalg.norm(np.array(world_pos) - np.array(pos)) < self.confirmation_radius:
                matched_key = key
                break

        if matched_key:
            self.pending_detections[matched_key]['times'].append(now)
        else:
            key = '%.2f_%.2f_%.2f' % (world_pos[0], world_pos[1], world_pos[2])
            self.pending_detections[key] = {
                'times': [now],
                'room_id': room_id
            }

        # 检查确认
        if matched_key:
            recent = [t for t in self.pending_detections[matched_key]['times']
                     if now - t < self.confirmation_valid_seconds]
            if len(recent) >= self.confirmation_threshold:
                pos = matched_key.split('_')
                pos = [float(pos[0]), float(pos[1]), float(pos[2])]
                if pos not in self.confirmed_dangers:
                    self.confirmed_dangers.append(pos)
                    rospy.loginfo("[危险源检测] 确认危险源[%s]: [%.2f, %.2f, %.2f]",
                                room_id, pos[0], pos[1], pos[2])
                del self.pending_detections[matched_key]

    def timer_callback(self, event):
        """定时输出检测结果"""
        exploration_time = (rospy.Time.now() - self.exploration_start_time).to_sec()

        output_data = {
            "exploration_time": round(exploration_time, 2),
            "detected_danger_sources": [
                {"position": [round(pos[0], 2), round(pos[1], 2), round(pos[2], 2)]}
                for pos in self.confirmed_dangers
            ]
        }

        os.makedirs(self.results_dir, exist_ok=True)
        output_file = os.path.join(self.results_dir, 'detected_danger.json')

        try:
            with open(output_file, 'w') as f:
                json.dump(output_data, f, indent=2)
            rospy.loginfo("[危险源检测] 已确认 %d 个危险源", len(self.confirmed_dangers))
        except Exception as e:
            rospy.logerr("[危险源检测] 保存失败: %s", str(e))

def main():
    rospy.init_node('danger_detection_node', anonymous=True)
    try:
        node = DangerDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()