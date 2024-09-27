#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose

class USVModelPublisher:
    def __init__(self):
        # Inicializa el nodo
        rospy.init_node('usv_model', anonymous=True)
        
        # Crea un publicador en el tópico '/usv/pose' de tipo 'Pose'
        self.pub = rospy.Publisher('/usv/pose', Pose, queue_size=10)
        
        # Define la frecuencia de publicación
        self.rate = rospy.Rate(400)  # 400 Hz
        self.x=0.0001
        self.x1=0

    def publish_pose(self):
        # Crea un mensaje Pose
        pose_msg = Pose()

        pose_msg.position.x = self.x1
        pose_msg.position.y = self.x1
        pose_msg.position.z = 1.0  
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0  
        self.x1=self.x1+self.x
        # Publica el mensaje
        self.pub.publish(pose_msg)

    def run(self):
        # Bucle principal
        while not rospy.is_shutdown():
            # Publica la pose en cada iteración
            self.publish_pose()

            # Espera hasta la siguiente iteración a 400 Hz
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Crea una instancia del publicador
        usv_publisher = USVModelPublisher()

        # Ejecuta el bucle de publicación
        usv_publisher.run()
    except rospy.ROSInterruptException:
        pass
