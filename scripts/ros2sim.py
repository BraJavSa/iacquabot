#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class ROS2Sim:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('ros2sim', anonymous=True)

        # Servicio para cambiar el estado del modelo en Gazebo
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Suscripción al tópico de Pose
        self.pose_sub = rospy.Subscriber('/usv2/pose', Pose, self.pose_callback)

        # Inicializa el mensaje ModelState
        self.model_state = ModelState()
        self.model_state.model_name = 'wamv2'  # Nombre del robot en Gazebo
        self.model_state.reference_frame = 'world'  # Referencia para el movimiento

    def pose_callback(self, msg):
        """
        Callback que se ejecuta cada vez que se recibe un mensaje en /usv/pose.
        Actualiza la posición y orientación del robot en Gazebo.
        """
        # Actualiza la posición y orientación del modelo con el mensaje recibido
        self.model_state.pose = msg

        # Llama al servicio para actualizar el estado del modelo en Gazebo
        try:
            result = self.set_model_state_srv(self.model_state)
            
        except rospy.ServiceException as e:
            pass

if __name__ == '__main__':
    try:
        # Instancia de la clase ROS2Sim
        ros2sim = ROS2Sim()

        # Mantiene el nodo en ejecución esperando mensajes en /usv/pose
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
