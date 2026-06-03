#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2


def read_ascii_ply_xyz(path):
    points = []
    with open(path, "r", encoding="ascii") as f:
        header = True
        for line in f:
            line = line.strip()
            if header:
                if line == "end_header":
                    header = False
                continue
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            points.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return points


def main():
    rospy.init_node("competition_overall_map")
    map_file = rospy.get_param("~map_file")
    map_topic = rospy.get_param("~map_topic", "/competition_overall_map")
    frame_id = rospy.get_param("~frame_id", "map")
    publish_rate = float(rospy.get_param("~publish_rate", 0.5))

    points = read_ascii_ply_xyz(map_file)
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    pub = rospy.Publisher(map_topic, PointCloud2, queue_size=1, latch=True)
    rate = rospy.Rate(publish_rate)

    rospy.loginfo("Loaded competition overall map: %s (%d points), topic=%s", map_file, len(points), map_topic)
    while not rospy.is_shutdown():
        msg = point_cloud2.create_cloud(rospy.Header(frame_id=frame_id, stamp=rospy.Time.now()), fields, points)
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
