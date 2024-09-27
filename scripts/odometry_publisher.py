#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher', anonymous=True)

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber('/wamv/sensors/position/p3d_wamv', Odometry, self.odom_callback)

        self.prev_time = rospy.Time.now()
        self.prev_position = None
        self.prev_orientation = None
        self.odom_msg = Odometry()

        # Variables para almacenar la última posición y orientación
        self.current_position = None
        self.current_orientation = None

    def odom_callback(self, data):
        # Actualizar la posición y orientación actuales
        self.current_position = np.array([data.pose.pose.position.x,
                                           data.pose.pose.position.y,
                                           data.pose.pose.position.z])
        self.current_orientation = data.pose.pose.orientation

    def calculate_odometry(self):
        current_time = rospy.Time.now()

        if self.prev_position is None:
            # Inicialización de posición y orientación anteriores
            self.prev_position = self.current_position
            self.prev_orientation = self.current_orientation
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).to_sec()

        if dt > 0:
            # Calcular velocidades lineales y angulares
            linear_velocity = (self.current_position - self.prev_position) / dt
            angular_velocity = self.calculate_angular_velocity(self.prev_orientation, self.current_orientation, dt)

            # Actualizar mensaje de odometría
            self.odom_msg.twist.twist.linear.x = linear_velocity[0]
            self.odom_msg.twist.twist.linear.y = linear_velocity[1]
            self.odom_msg.twist.twist.linear.z = linear_velocity[2]

            self.odom_msg.twist.twist.angular.x = angular_velocity[0]
            self.odom_msg.twist.twist.angular.y = angular_velocity[1]
            self.odom_msg.twist.twist.angular.z = angular_velocity[2]

            self.odom_msg.pose.pose.position.x = self.current_position[0]
            self.odom_msg.pose.pose.position.y = self.current_position[1]
            self.odom_msg.pose.pose.position.z = self.current_position[2]
            self.odom_msg.pose.pose.orientation = self.current_orientation

            # Publicar mensaje de odometría
            self.odom_pub.publish(self.odom_msg)

        # Actualizar variables anteriores
        self.prev_position = self.current_position
        self.prev_orientation = self.current_orientation
        self.prev_time = current_time

    def calculate_angular_velocity(self, prev_orientation, current_orientation, dt):
        prev_rotation_matrix = self.quaternion_to_rotation_matrix(prev_orientation)
        current_rotation_matrix = self.quaternion_to_rotation_matrix(current_orientation)

        rotation_change = np.dot(np.linalg.inv(prev_rotation_matrix), current_rotation_matrix)

        angle = np.arccos((np.trace(rotation_change) - 1) / 2)
        if angle > 1e-6:
            axis = np.array([rotation_change[2, 1] - rotation_change[1, 2],
                             rotation_change[0, 2] - rotation_change[2, 0],
                             rotation_change[1, 0] - rotation_change[0, 1]]) / (2 * np.sin(angle))
            angular_velocity = axis * angle / dt
        else:
            angular_velocity = np.zeros(3)

        return angular_velocity

    def quaternion_to_rotation_matrix(self, quaternion):
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return np.array([[1 - 2 * q[1] ** 2 - 2 * q[2] ** 2, 2 * q[0] * q[1] - 2 * q[2] * q[3], 2 * q[0] * q[2] + 2 * q[1] * q[3]],
                         [2 * q[0] * q[1] + 2 * q[2] * q[3], 1 - 2 * q[0] ** 2 - 2 * q[2] ** 2, 2 * q[1] * q[2] - 2 * q[0] * q[3]],
                         [2 * q[0] * q[2] - 2 * q[1] * q[3], 2 * q[1] * q[2] + 2 * q[0] * q[3], 1 - 2 * q[0] ** 2 - 2 * q[1] ** 2]])

    def publish_odometry(self):
        rate = rospy.Rate(20)  # Publicar a 100 Hz

        while not rospy.is_shutdown():
            self.calculate_odometry()
            # Encabezado del mensaje de odometría
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.header.frame_id = 'map'
            self.odom_msg.child_frame_id = 'wamv/base_link'

            rate.sleep()

if __name__ == '__main__':
    try:
        odom_publisher = OdometryPublisher()
        odom_publisher.publish_odometry()
    except rospy.ROSInterruptException:
        pass
