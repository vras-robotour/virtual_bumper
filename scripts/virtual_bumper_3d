#!/usr/bin/env python
"""Virtual bumper using local point clouds in 3D.
"""
from __future__ import absolute_import, division, print_function
from geometry_msgs.msg import (Point, Pose, PoseStamped, Transform, TransformStamped, Twist, Vector3)
from nav_msgs.msg import Path
import numpy as np
import rospy
from ros_numpy import msgify, numpify
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
import tf2_ros
from threading import RLock
from timeit import default_timer as timer
import traceback
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import (euler_from_matrix, euler_matrix)

np.set_printoptions(precision=2)


class VirtualBumper(object):
    def __init__(self):
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.max_age = rospy.get_param('~max_age', 1.0)
        clearance_box = rospy.get_param('~clearance_box', [[-0.6, 0.6],
                                                           [-0.5, 0.5],
                                                           [ 0.0, 0.8]])
        self.clearance_box = np.array(clearance_box)
        self.min_points_obstacle = rospy.get_param('~min_points_obstacle', 1)
        self.keep_clouds = int(rospy.get_param('~keep_clouds', 1))

        self.cloud_lock = RLock()
        self.clouds = [None] * self.keep_clouds
        self.i_cloud = 0
        self.blocked = False

        self.cmd_pub = rospy.Publisher('output_cmd_vel', Twist, queue_size=2)
        self.cloud_pub = rospy.Publisher('obstacle_cloud', PointCloud2, queue_size=2)
        self.markers_pub = rospy.Publisher('~markers', MarkerArray, queue_size=2)

        self.cmd_sub = rospy.Subscriber('input_cmd_vel', Twist, self.receive_cmd, queue_size=2)
        self.cloud_sub = rospy.Subscriber('input_cloud', PointCloud2, self.receive_cloud, queue_size=2)

    def receive_cloud(self, msg):
        t = timer()
        assert isinstance(msg, PointCloud2)
        assert msg.header.frame_id == self.robot_frame

        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        if age > self.max_age:
            rospy.logwarn('Discarding cloud %.1f s > %.1f s old.', age, self.max_age)
            return

        cloud = numpify(msg)
        
        with self.cloud_lock:
            i = self.i_cloud % self.keep_clouds
            self.clouds[i] = cloud
            self.i_cloud += 1
        t = timer() - t
        rospy.logdebug(f'{cloud.shape[0]} points received in slot {i} ({t:.3f} s).')

    def obstacle_cloud(self, cloud):
        obstacles = np.ones(cloud.shape[0], dtype=bool)
        for i, (f_min, f_max) in enumerate(zip(self.clearance_box[:, 0], self.clearance_box[:, 1])):
            obstacles &= (cloud[:, i] >= f_min) & (cloud[:, i] <= f_max)

        obstacle_cloud = cloud[obstacles]
        return obstacle_cloud
    
    def full_obstacle_cloud(self):
        obstacle_clouds = []
        with self.cloud_lock:
            for cloud in self.clouds:
                if cloud is None:
                    rospy.logwarn_throttle(1.0, 'Missing input clouds to check clearance.')
                    continue
                obstacle_cloud = self.obstacle_cloud(cloud)
                obstacle_clouds.append(obstacle_cloud)

        if not obstacle_clouds:
            return None
        
        obstacle_cloud = np.concatenate(obstacle_clouds)
        return obstacle_cloud

    def check_clearance(self):
        if self.min_points_obstacle < 1:
            return True, None
        
        obstacle_cloud = self.full_obstacle_cloud()

        if obstacle_cloud is None:
            return True, None
        
        free = obstacle_cloud.shape[0] < self.min_points_obstacle
        return free, obstacle_cloud

    def clearance_ns(self):
        return f'{self.robot_frame}/clearance'

    def clearance_marker(self, free):
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.clearance_ns()
        marker.id = 0
        marker.action = Marker.MODIFY
        marker.type = Marker.CUBE
        marker.pose.orientation.w = 1.
        marker.scale = Vector3(*(self.clearance_box[:, 1] - self.clearance_box[:, 0]))
        if free:
            marker.color = ColorRGBA(0., 1., 0., 0.25)
        else:
            marker.color = ColorRGBA(1., 0., 0., 0.5)
        marker.frame_locked = True
        marker.lifetime = rospy.Duration(1.0)
        return marker

    def publish_clearance_marker(self, free):
        t = timer()
        msg = MarkerArray([self.clearance_marker(free)])
        self.markers_pub.publish(msg)
        t = timer() - t
        rospy.logdebug(f'Publish clearance: {t:.3f} s.')
    
    def publish_obstacle_cloud(self, cloud):
        t = timer()
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = self.robot_frame
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.height = 1
        cloud_msg.width = cloud.shape[0]
        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = cloud.astype(np.float32).tobytes()
        self.cloud_pub.publish(cloud_msg)
        t = timer() - t
        rospy.logdebug(f'Publish obstacle cloud: {t:.3f} s.')

    def receive_cmd(self, msg):
        assert isinstance(msg, Twist)

        # Check clearance.
        free, obstacle_cloud = self.check_clearance()

        # Pass through or stop.
        if free:
            self.cmd_pub.publish(msg)
            if self.blocked:
                rospy.logwarn('Path is clear.')
            self.blocked = False
        else:
            self.cmd_pub.publish(Twist())
            if not self.blocked:
                rospy.logwarn('Blocked by obstacle.')
            self.blocked = True

        # Publish clearance markers.
        self.publish_clearance_marker(free)
        if obstacle_cloud is not None:
            self.publish_obstacle_cloud(obstacle_cloud)

if __name__ == '__main__':
    rospy.init_node('virtual_bumper', log_level=rospy.INFO)
    node = VirtualBumper()
    rospy.spin()
