#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion

class ImuPublisher:
    def __init__(self):
        rospy.init_node('imu_publisher', anonymous=True)

        self.imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
        rospy.Subscriber('/wamv/sensors/position/p3d_wamv', Odometry, self.odom_callback)

        self.prev_time = rospy.Time.now()
        self.prev_position = None
        self.prev_orientation = None

        self.imu_msg = Imu()

    def odom_callback(self, data):
        current_time = rospy.Time.now()

        if self.prev_position is None:
            self.prev_position = np.array([data.pose.pose.position.x,
                                           data.pose.pose.position.y,
                                           data.pose.pose.position.z])
            self.prev_orientation = data.pose.pose.orientation
            self.prev_time = current_time
            return

        current_orientation = data.pose.pose.orientation
        current_position = np.array([data.pose.pose.position.x,
                                     data.pose.pose.position.y,
                                     data.pose.pose.position.z])

        dt = (current_time - self.prev_time).to_sec()

        if dt > 0:
            linear_velocity = (current_position - self.prev_position) / dt

            if self.prev_position is not None:
                prev_velocity = (self.prev_position - self.prev_position) / dt  # Placeholder for previous velocity
                linear_acceleration = (linear_velocity - prev_velocity) / dt
            else:
                linear_acceleration = np.array([0, 0, 0])

            angular_velocity = self.calculate_angular_velocity(self.prev_orientation, current_orientation, dt)

            self.imu_msg.linear_acceleration.x = linear_acceleration[0]
            self.imu_msg.linear_acceleration.y = linear_acceleration[1]
            self.imu_msg.linear_acceleration.z = linear_acceleration[2]

            self.imu_msg.angular_velocity.x = angular_velocity[0]
            self.imu_msg.angular_velocity.y = angular_velocity[1]
            self.imu_msg.angular_velocity.z = angular_velocity[2]

            self.imu_msg.orientation = current_orientation

            self.imu_pub.publish(self.imu_msg)

        self.prev_position = current_position
        self.prev_orientation = current_orientation
        self.prev_time = current_time

    def calculate_angular_velocity(self, prev_orientation, current_orientation, dt):
        prev_rotation_matrix = self.quaternion_to_rotation_matrix(prev_orientation)
        current_rotation_matrix = self.quaternion_to_rotation_matrix(current_orientation)

        rotation_change = np.dot(np.linalg.inv(prev_rotation_matrix), current_rotation_matrix)

        angle = math.acos((np.trace(rotation_change) - 1) / 2)

        if angle > 1e-6:
            axis = np.array([rotation_change[2, 1] - rotation_change[1, 2],
                             rotation_change[0, 2] - rotation_change[2, 0],
                             rotation_change[1, 0] - rotation_change[0, 1]]) / (2 * math.sin(angle))
            angular_velocity = axis * angle / dt
        else:
            angular_velocity = np.zeros(3)

        return angular_velocity

    def quaternion_to_rotation_matrix(self, quaternion):
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return np.array([[1 - 2 * q[1] ** 2 - 2 * q[2] ** 2, 2 * q[0] * q[1] - 2 * q[2] * q[3], 2 * q[0] * q[2] + 2 * q[1] * q[3]],
                         [2 * q[0] * q[1] + 2 * q[2] * q[3], 1 - 2 * q[0] ** 2 - 2 * q[2] ** 2, 2 * q[1] * q[2] - 2 * q[0] * q[3]],
                         [2 * q[0] * q[2] - 2 * q[1] * q[3], 2 * q[1] * q[2] + 2 * q[0] * q[3], 1 - 2 * q[0] ** 2 - 2 * q[1] ** 2]])

    def publish_imu(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.imu_msg.header = Header()
            self.imu_msg.header.stamp = rospy.Time.now()
            self.imu_msg.header.frame_id = 'base_link'

            rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher = ImuPublisher()
        imu_publisher.publish_imu()
    except rospy.ROSInterruptException:
        pass
