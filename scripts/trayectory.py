#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

class DroneControl:
    def __init__(self):
        # Inicialización de ROS
        rospy.init_node('drone_control_node', anonymous=True)
        
        # Publicador
        self.pub_controls = rospy.Publisher('/controls', Float32MultiArray, queue_size=10)

        # Frecuencia de actualización
        self.rate = rospy.Rate(400)  # 10 Hz

    def move_drone(self):
        start_time = rospy.get_time()  # Usar get_time para un tiempo más preciso
        
        while not rospy.is_shutdown():
            t = rospy.get_time() - start_time

            # Generar una trayectoria arbitraria
            u1 = 9.81  # Fuerza total para elevación
            u2 = 0.05 * np.sin(2 * np.pi * t / 2)        # Momento alrededor del eje X
            u3 = 0.05 * np.cos(2 * np.pi * t / 2)        # Momento alrededor del eje Y
            u4 = 0.05 * np.sin(4 * np.pi * t / 2)        # Momento alrededor del eje Z

            # Crear el mensaje de control
            control_msg = Float32MultiArray()
            control_msg.data = [u1, u2, u3, u4]

            # Publicar el mensaje en /controls
            self.pub_controls.publish(control_msg)

            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        drone_control = DroneControl()
        drone_control.move_drone()
    except rospy.ROSInterruptException:
        pass
