#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class JoyTeleop:
    def __init__(self):
        rospy.init_node('joy_teleop')

        # Suscribirse al topic del joystick
        rospy.Subscriber("/joy0", Joy, self.joy_callback)

        # Publicar al topic de controles
        self.control_pub = rospy.Publisher('/controls', Float32MultiArray, queue_size=10)

        # Inicializar valores anteriores de los ejes del joystick
        self.prev_axes = [0.0, 0.0, 0.0, 0.0]

        # Frecuencia de publicación (en Hz)
        self.publish_rate = rospy.Rate(100)  # Publica cada 0.1 segundos

    def joy_callback(self, joy_msg):
        # Actualizar los valores de los ejes con los datos recibidos
        self.prev_axes = [
            joy_msg.axes[1]+9.81,  # eje vertical izquierdo (arriba/abajo)
            joy_msg.axes[0]*3,  # eje horizontal izquierdo (izquierda/derecha)
            joy_msg.axes[4]*3,  # botón RT (adelante/atrás)
            joy_msg.axes[3]*3,  # eje horizontal derecho (izquierda/derecha)
        ]

    def control_callback(self):
        # Crear el mensaje Float32MultiArray con los valores actuales de los ejes
        control_msg = Float32MultiArray()
        control_msg.data = self.prev_axes  # Usar los valores actuales o anteriores

        # Publicar el mensaje
        self.control_pub.publish(control_msg)

    def run(self):
        while not rospy.is_shutdown():
            # Llamar al método de publicación a la frecuencia deseada
            self.control_callback()
            self.publish_rate.sleep()

if __name__ == '__main__':
    joy_teleop = JoyTeleop()
    joy_teleop.run()
