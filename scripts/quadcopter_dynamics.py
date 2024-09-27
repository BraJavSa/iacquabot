#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray

class QuadcopterDynamics:
    def __init__(self):
        rospy.init_node('quadcopter_dynamics_node')

        # Parámetros del quadcóptero
        self.m = 1.0  # Masa del quadcóptero en kg
        self.I_x = 0.1  # Inercia en x
        self.I_y = 0.1  # Inercia en y
        self.I_z = 0.2  # Inercia en z
        self.g = 9.81  # Aceleración debida a la gravedad (m/s^2)

        self.dt = 0.1  # Intervalo de tiempo para la simulación
        self.state_dt = 0.05  # Intervalo de tiempo para la publicación del estado (50 ms)

        # Estado inicial [x, y, z, roll, pitch, yaw]
        self.eta = np.array([0.0, 0.0, 20.0, 0.0, 0.0, 0.0])  # Inicializa a 20 unidades en z
        self.nu = np.zeros(6)  # Velocidades [u, v, w, p, q, r]

        # Definición de la matriz de masa M (6x6)
        self.M = np.diag([self.m, self.m, self.m, self.I_x, self.I_y, self.I_z])  # Matriz de masa diagonal

        # Publicador
        self.pose_pub = rospy.Publisher('quadcopter/pose', Pose, queue_size=10)

        # Suscriptor para acciones de control
        rospy.Subscriber('quadcopter/control', Float64MultiArray, self.control_callback)

        # Inicializar variables de control
        self.tau = np.zeros(6)

    def control_callback(self, msg):
        self.tau = np.array(msg.data)

    def simulate_dynamics(self):
        # Matriz de Coriolis C
        u, v, w, p, q, r = self.nu
        C = np.zeros((6, 6))
        C[3, 4] = -self.m * w
        C[3, 5] = self.m * v
        C[4, 3] = self.m * w
        C[5, 3] = -self.m * v

        # Matriz de resistencia aerodinámica D
        D = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01])

        # Fuerzas gravitacionales
        g_eta = np.array([0, 0, self.m * self.g, 0, 0, 0])

        # Ecuación de movimiento: M * dnu/dt + C * nu + D * nu = tau + g(eta)
        dnu = np.linalg.inv(self.M).dot(self.tau + g_eta - C.dot(self.nu) - D.dot(self.nu))

        # Actualización de las velocidades
        self.nu += dnu * self.dt

        # Actualización del estado
        self.eta += self.nu * self.dt

        # Publicación de la posición
        pose_msg = Pose()
        pose_msg.position.x = self.eta[0]
        pose_msg.position.y = self.eta[1]
        pose_msg.position.z = self.eta[2]

        # Publicar la posición
        self.pose_pub.publish(pose_msg)

    def run(self):
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():
            self.simulate_dynamics()
            rate.sleep()

if __name__ == '__main__':
    quadcopter = QuadcopterDynamics()
    quadcopter.run()
