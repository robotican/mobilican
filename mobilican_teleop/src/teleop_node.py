#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

axis_linear = 3
axis_angular = 2
deadman_button = 6
reverse_linear = False
reverse_angular = False
twist_topic = "mobile_base_controller/cmd_vel"

def joyCallback(msg):

    #print (msg)

    twist_msg = Twist()
    if msg.buttons[deadman_button]:
        twist_msg.linear.x = msg.axes[axis_linear]
        if reverse_linear:
            twist_msg.linear.x *= -1

        twist_msg.angular.z = msg.axes[axis_angular]
        if reverse_angular:
            twist_msg.angular.z *= -1
    else:
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        #TODO: send emergency stop to roboteq

    twist_pub.publish(twist_msg)



if __name__ == '__main__':

    rospy.init_node('teleop_node', anonymous=False)

    rospy.get_param('~axis_linear', axis_linear)
    rospy.get_param('~axis_angular', axis_angular)
    rospy.get_param('~deadman_button', deadman_button)
    rospy.get_param('~reverse_angular', reverse_angular)
    rospy.get_param('~reverse_linear', reverse_linear)
    rospy.get_param('~twist_topic', twist_topic)

    rospy.Subscriber("/joy", Joy, joyCallback)
    twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=10)

    rospy.spin()
