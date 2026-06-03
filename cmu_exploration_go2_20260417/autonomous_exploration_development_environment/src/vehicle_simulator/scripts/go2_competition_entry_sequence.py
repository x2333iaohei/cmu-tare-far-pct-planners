#!/usr/bin/env python3

import math
import os
import sys

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

try:
    from building_generator_interfaces.srv import SetDoorState
except ImportError:
    simenv_python_path = os.environ.get(
        "SIMENV_PYTHON_PATH",
        "/home/xiaohei/揭榜挂帅/SimEnv/devel/lib/python3/dist-packages",
    )
    if os.path.isdir(simenv_python_path):
        sys.path.insert(0, simenv_python_path)
    try:
        from building_generator_interfaces.srv import SetDoorState
    except ImportError:
        SetDoorState = None


class Go2CompetitionEntrySequence:
    def __init__(self):
        self.door_id = rospy.get_param("~door_id", "main_entrance")
        self.door_model = rospy.get_param("~door_model", "dynamic_main_entrance")
        self.door_x = rospy.get_param("~door_x", 0.0)
        self.door_y = rospy.get_param("~door_y", 0.0)
        self.door_z = rospy.get_param("~door_z", 1.2)
        self.door_yaw = rospy.get_param("~door_yaw", 1.5707963267948966)
        self.prefer_door_service = rospy.get_param("~prefer_door_service", False)
        self.open_on_start = rospy.get_param("~open_on_start", True)
        self.close_after_entry = rospy.get_param("~close_after_entry", True)
        self.direct_drive = rospy.get_param("~direct_drive", False)
        self.entry_x = rospy.get_param("~entry_x", 0.0)
        self.entry_y = rospy.get_param("~entry_y", 2.0)
        self.entry_z = rospy.get_param("~entry_z", 0.6)
        self.entry_reached_y = rospy.get_param("~entry_reached_y", 1.45)
        self.entry_reached_radius = rospy.get_param("~entry_reached_radius", 0.8)
        self.waypoint_rate = rospy.get_param("~waypoint_rate", 5.0)
        self.entry_speed = rospy.get_param("~entry_speed", 0.25)
        self.heading_kp = rospy.get_param("~heading_kp", 0.8)
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 0.25)
        self.start_delay = rospy.get_param("~start_delay", 3.0)
        self.require_clouds_before_start = rospy.get_param("~require_clouds_before_start", False)
        self.cloud_wait_timeout = rospy.get_param("~cloud_wait_timeout", 20.0)
        self.progress_warn_period = rospy.get_param("~progress_warn_period", 6.0)
        self.progress_min_delta_y = rospy.get_param("~progress_min_delta_y", 0.05)
        self.keep_alive = rospy.get_param("~keep_alive", True)
        self.frame_id = rospy.get_param("~frame_id", "map")

        self.pose = None
        self.yaw = None
        self.registered_scan_ready = False
        self.terrain_map_ready = False
        self.waypoint_pub = rospy.Publisher("/way_point", PointStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=1)
        self.speed_pub = rospy.Publisher("/speed", Float32, queue_size=1)
        self.start_pub = rospy.Publisher("/start_exploration", Bool, queue_size=1, latch=True)
        self.door_service = (
            rospy.ServiceProxy("/set_door_state", SetDoorState)
            if self.prefer_door_service and SetDoorState is not None
            else None
        )
        rospy.Subscriber("/state_estimation", Odometry, self.odom_cb, queue_size=5)
        rospy.Subscriber("/registered_scan", PointCloud2, self.registered_scan_cb, queue_size=1)
        rospy.Subscriber("/terrain_map", PointCloud2, self.terrain_map_cb, queue_size=1)

    def odom_cb(self, msg):
        self.pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

    def registered_scan_cb(self, msg):
        self.registered_scan_ready = msg.width * msg.height > 0

    def terrain_map_cb(self, msg):
        self.terrain_map_ready = msg.width * msg.height > 0

    def set_door(self, open_state):
        if self.door_service is not None:
            try:
                rospy.wait_for_service("/set_door_state", timeout=1.0)
                response = self.door_service(door_id=self.door_id, open=open_state)
                if not response.accepted:
                    raise RuntimeError(response.message)
                rospy.loginfo("Door %s set to %s: %s", self.door_id, "open" if open_state else "closed", response.message)
                return True
            except Exception as exc:
                rospy.logwarn("Door service unavailable, falling back to Gazebo link states: %s", exc)
        elif self.prefer_door_service:
            rospy.logwarn("Door service type unavailable, falling back to Gazebo link states")
        return self.set_door_links(open_state)

    def set_door_links(self, open_state):
        from gazebo_msgs.msg import LinkState
        from gazebo_msgs.srv import SetLinkState

        rospy.wait_for_service("/gazebo/set_link_state")
        set_link_state = rospy.ServiceProxy("/gazebo/set_link_state", SetLinkState)
        panel_y = 1.06 if open_state else 0.5
        success = True
        for link_name, y in (("left_panel", -panel_y), ("right_panel", panel_y)):
            wx, wy = self.transform_door_point(0.0, y)
            q = quaternion_from_euler(0.0, 0.0, self.door_yaw)
            state = LinkState()
            state.link_name = "%s::%s" % (self.door_model, link_name)
            state.reference_frame = "world"
            state.pose.position.x = wx
            state.pose.position.y = wy
            state.pose.position.z = self.door_z
            state.pose.orientation.x = q[0]
            state.pose.orientation.y = q[1]
            state.pose.orientation.z = q[2]
            state.pose.orientation.w = q[3]
            result = set_link_state(state)
            if not result.success:
                success = False
                rospy.logerr(
                    "Failed to set %s in world frame at (%.3f, %.3f, %.3f): %s",
                    state.link_name,
                    wx,
                    wy,
                    self.door_z,
                    result.status_message,
                )
        if success:
            rospy.loginfo("Door %s set to %s through Gazebo link states", self.door_model, "open" if open_state else "closed")
        return success

    def transform_door_point(self, rel_x, rel_y):
        c = math.cos(self.door_yaw)
        s = math.sin(self.door_yaw)
        return self.door_x + c * rel_x - s * rel_y, self.door_y + s * rel_x + c * rel_y

    def publish_entry_waypoint(self):
        if self.direct_drive:
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "body"
            cmd.twist.linear.x = self.entry_speed
            cmd.twist.angular.z = self.entry_yaw_rate()
            self.cmd_vel_pub.publish(cmd)
        else:
            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.point.x = self.entry_x
            msg.point.y = self.entry_y
            msg.point.z = self.entry_z
            self.waypoint_pub.publish(msg)
            self.speed_pub.publish(Float32(data=self.entry_speed))

    def reached_entry(self):
        if self.pose is None:
            return False
        dx = self.pose.x - self.entry_x
        dy = self.pose.y - self.entry_y
        return self.pose.y >= self.entry_reached_y and math.hypot(dx, dy) <= max(0.01, self.entry_reached_radius)

    def entry_yaw_rate(self):
        if self.pose is None or self.yaw is None:
            return 0.0
        target_yaw = math.atan2(self.entry_y - self.pose.y, self.entry_x - self.pose.x)
        error = self.normalize_angle(target_yaw - self.yaw)
        return max(-self.max_yaw_rate, min(self.max_yaw_rate, self.heading_kp * error))

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def wait_for_clouds(self):
        if not self.require_clouds_before_start:
            return
        deadline = rospy.Time.now() + rospy.Duration.from_sec(self.cloud_wait_timeout)
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.registered_scan_ready and self.terrain_map_ready:
                rospy.loginfo("Registered scan and terrain map are ready; starting TARE")
                return
            rospy.loginfo(
                "Waiting for planner clouds before /start_exploration: registered_scan=%s terrain_map=%s",
                self.registered_scan_ready,
                self.terrain_map_ready,
            )
            rate.sleep()
        rospy.logwarn("Planner clouds not ready before timeout; publishing /start_exploration anyway")
        return

    def run(self):
        if self.open_on_start:
            if not self.set_door(True):
                rospy.logwarn("Continuing entry sequence after open-door command failed; world may already be open")

        rospy.loginfo(
            "Publishing entry waypoint (%.2f, %.2f, %.2f) until GO2 reaches y>=%.2f or radius<=%.2f",
            self.entry_x,
            self.entry_y,
            self.entry_z,
            self.entry_reached_y,
            self.entry_reached_radius,
        )

        rate = rospy.Rate(self.waypoint_rate)
        last_progress_check = rospy.Time.now()
        last_progress_y = self.pose.y if self.pose is not None else None
        while not rospy.is_shutdown() and not self.reached_entry():
            self.publish_entry_waypoint()
            now = rospy.Time.now()
            if (
                self.pose is not None
                and last_progress_y is not None
                and (now - last_progress_check).to_sec() >= self.progress_warn_period
            ):
                delta_y = self.pose.y - last_progress_y
                if delta_y < self.progress_min_delta_y:
                    rospy.logwarn(
                        "GO2 entry progress is low: delta_y=%.3f in %.1fs. Check junior_ctrl is in RL mode: press 2 then 6, and local_planner is publishing /cmd_vel.",
                        delta_y,
                        self.progress_warn_period,
                    )
                last_progress_check = now
                last_progress_y = self.pose.y
            elif self.pose is not None and last_progress_y is None:
                last_progress_y = self.pose.y
                last_progress_check = now
            rate.sleep()

        if rospy.is_shutdown():
            return

        if self.close_after_entry:
            self.set_door(False)
        elif self.direct_drive:
            stop = TwistStamped()
            stop.header.stamp = rospy.Time.now()
            stop.header.frame_id = "body"
            self.cmd_vel_pub.publish(stop)

        if self.start_delay > 0:
            rospy.sleep(self.start_delay)

        self.wait_for_clouds()
        self.start_pub.publish(Bool(data=True))
        rospy.loginfo("GO2 entry sequence complete; /start_exploration published")

        if self.keep_alive:
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node("go2_competition_entry_sequence")
    try:
        Go2CompetitionEntrySequence().run()
    except Exception as exc:
        rospy.logerr("GO2 competition entry sequence failed: %s", exc)
        raise
