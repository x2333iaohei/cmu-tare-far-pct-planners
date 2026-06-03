#!/usr/bin/env python3

import math

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from tf.transformations import quaternion_multiply


def rotate_xy(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


class Go2CompetitionFrameBridge:
    def __init__(self):
        self.offset_x = rospy.get_param("~offset_x", 0.0)
        self.offset_y = rospy.get_param("~offset_y", -2.1)
        self.offset_z = rospy.get_param("~offset_z", 0.6)
        self.offset_yaw = rospy.get_param("~offset_yaw", 1.5708)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "body")

        half_yaw = 0.5 * self.offset_yaw
        self.q_offset = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

        self.odom_pub = rospy.Publisher("/state_estimation", Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher("/registered_scan", PointCloud2, queue_size=2)

        rospy.Subscriber("/fastlio/state_estimation_raw", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber("/fastlio/registered_scan_raw", PointCloud2, self.cloud_cb, queue_size=2)

        rospy.loginfo(
            "GO2 competition frame bridge: offset=(%.3f, %.3f, %.3f), yaw=%.4f",
            self.offset_x,
            self.offset_y,
            self.offset_z,
            self.offset_yaw,
        )

    def transform_position(self, x, y, z):
        rx, ry = rotate_xy(x, y, self.offset_yaw)
        return rx + self.offset_x, ry + self.offset_y, z + self.offset_z

    def odom_cb(self, msg):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.child_frame
        out.pose = msg.pose
        out.twist = msg.twist

        p = msg.pose.pose.position
        x, y, z = self.transform_position(p.x, p.y, p.z)
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = z

        q = msg.pose.pose.orientation
        q_new = quaternion_multiply(self.q_offset, (q.x, q.y, q.z, q.w))
        out.pose.pose.orientation.x = q_new[0]
        out.pose.pose.orientation.y = q_new[1]
        out.pose.pose.orientation.z = q_new[2]
        out.pose.pose.orientation.w = q_new[3]

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        rvx, rvy = rotate_xy(vx, vy, self.offset_yaw)
        out.twist.twist.linear.x = rvx
        out.twist.twist.linear.y = rvy
        out.twist.twist.linear.z = vz

        self.odom_pub.publish(out)

    def cloud_cb(self, msg):
        field_names = [field.name for field in msg.fields]
        transformed = []
        for point in point_cloud2.read_points(msg, field_names=field_names, skip_nans=True):
            values = list(point)
            try:
                xi = field_names.index("x")
                yi = field_names.index("y")
                zi = field_names.index("z")
            except ValueError:
                return
            x, y, z = self.transform_position(values[xi], values[yi], values[zi])
            values[xi] = x
            values[yi] = y
            values[zi] = z
            transformed.append(values)

        out = point_cloud2.create_cloud(msg.header, msg.fields, transformed)
        out.header.frame_id = self.map_frame
        self.cloud_pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("go2_competition_frame_bridge")
    Go2CompetitionFrameBridge()
    rospy.spin()
