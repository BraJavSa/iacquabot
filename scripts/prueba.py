#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray  # Importar el mensaje correcto
import numpy as np

class DroneModel:
    def __init__(self):
        # Inicialización de ROS
        rospy.init_node('drone_model_node', anonymous=True)
        
        # Parámetros del dron
        self.gravity = 9.81  # Aceleración debido a la gravedad
        self.dt = 0.01        # Intervalo de tiempo de actualización (en segundos)
        
        # Estado del dron: [x, y, z, roll (phi), pitch (theta), yaw (psi)]
        self.state = np.zeros(6)  # Inicializado en [0, 0, 0, 0, 0, 0]

        # Controles iniciales (todos en cero)
        self.u = np.zeros(4)  # [u1, u2, u3, u4]

        # Suscriptores
        self.sub_controls = rospy.Subscriber('/controls', Float32MultiArray, self.control_callback)  # Cambiar a Float32MultiArray
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publicador
        self.pub_pose = rospy.Publisher('/usv/pose', Pose, queue_size=10)

        # Tasa de actualización
        self.rate = rospy.Rate(1 / self.dt)  # Frecuencia de actualización

    def control_callback(self, msg):
        # Asume que msg es un array de control [u1, u2, u3, u4]
        self.u = np.array(msg.data)  # Actualiza los controles cuando se recibe un nuevo mensaje

    def odom_callback(self, msg):
        # Actualiza el estado basado en la odometría
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = msg.pose.pose.position.z
        orientation = msg.pose.pose.orientation
        self.state[3] = orientation.x  # Roll
        self.state[4] = orientation.y  # Pitch
        self.state[5] = orientation.z  # Yaw

    def update_state(self):
        # Descomponer el estado
        x, y, z, phi, theta, psi = self.state
        
        # Cálculo de velocidades
        v_x = self.u[0] * np.sin(theta) * np.cos(phi) + self.u[3]
        v_y = self.u[0] * np.sin(theta) * np.sin(phi) - self.u[2]
        v_z = self.u[0] * np.cos(theta) - self.gravity
        
        # Cálculo de velocidades angulares
        omega_x = self.u[1]
        omega_y = self.u[2]
        omega_z = self.u[3]
        
        # Actualización del estado
        self.state[0] += v_x * self.dt  # Actualiza x
        self.state[1] += v_y * self.dt  # Actualiza y
        self.state[2] += v_z * self.dt  # Actualiza z
        self.state[3] += omega_x * self.dt  # Actualiza roll
        self.state[4] += omega_y * self.dt  # Actualiza pitch
        self.state[5] += omega_z * self.dt  # Actualiza yaw

    def publish_pose(self):
        pose_msg = Pose()
        pose_msg.position.x = self.state[0]
        pose_msg.position.y = self.state[1]
        if self.state[2] > -0.05:
            pose_msg.position.z = self.state[2]
        else:
            pose_msg.position.z = -0.05
        # Para la orientación, se necesita convertir los ángulos de Euler a cuaterniones
        quaternion = self.euler_to_quaternion(self.state[3], self.state[4], self.state[5])
        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]

        self.pub_pose.publish(pose_msg)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        # Conversión de ángulos de Euler (roll, pitch, yaw) a cuaterniones
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([x, y, z, w])

    def run(self):
        while not rospy.is_shutdown():
            self.update_state()
            self.publish_pose()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        drone_model = DroneModel()
        drone_model.run()
    except rospy.ROSInterruptException:
        pass
