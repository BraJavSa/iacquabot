#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def handle_odom(msg):
    # Crear un objeto de TransformBroadcaster de tf2_ros
    br = tf2_ros.TransformBroadcaster()

    # Crear un mensaje de tipo TransformStamped
    t = geometry_msgs.msg.TransformStamped()

    # Configurar el marco padre e hijo
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"  # Marco padre
    t.child_frame_id = "wamv/base_link"  # Marco hijo

    # Asignar la traslación (posición) a partir del mensaje de odometría
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    # Asignar la rotación (orientación) a partir del mensaje de odometría
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    # Publicar la transformación
    br.sendTransform(t)

def odom_tf_broadcaster():
    rospy.init_node('odom_tf_broadcaster')

    # Suscribirse al tópico de odometría
    rospy.Subscriber('/odom', Odometry, handle_odom)

    # Mantener el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        odom_tf_broadcaster()
    except rospy.ROSInterruptException:
        pass
