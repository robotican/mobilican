#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

angular_axis = 2
angular_reverse = False
angular_clamp = 0
angular_multiplier = 1

linear_axis = 3
linear_reverse = False
linear_clamp = 0
linear_multiplier = 1

deadman_button = 6
twist_topic = "mobile_base_controller/cmd_vel"

def joyCallback(msg):

    #print (msg)

    twist_msg = Twist()
    if msg.buttons[deadman_button]:
        twist_msg.linear.x = msg.axes[linear_axis] * linear_multiplier
        if linear_reverse:
            twist_msg.linear.x *= -1

        # linear saturation
        if twist_msg.linear.x > 0:
            if twist_msg.linear.x > linear_clamp:
                twist_msg.linear.x = linear_clamp
        else:
            if twist_msg.linear.x < 0:
                if twist_msg.linear.x < -linear_clamp:
                    twist_msg.linear.x = -linear_clamp

        twist_msg.angular.z = msg.axes[angular_axis] * angular_multiplier
        if angular_reverse:
            twist_msg.angular.z *= -1

        # angular saturation
        if twist_msg.linear.z > 0:
            if twist_msg.linear.z > angular_clamp:
                twist_msg.angular.z = angular_clamp
        else:
            if twist_msg.linear.z < 0:
                if twist_msg.linear.z < -angular_clamp:
                    twist_msg.linear.z = -angular_clamp

    else:
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0

    twist_pub.publish(twist_msg)



if __name__ == '__main__':

    rospy.init_node('teleop_node', anonymous=False)

    rospy.get_param('~angular_axis', angular_axis)
    rospy.get_param('~angular_reverse', angular_reverse)
    rospy.get_param('~angular_clamp', angular_clamp)
    rospy.get_param('~angular_multiplier', angular_multiplier)

    rospy.get_param('~linear_axis', linear_axis)
    rospy.get_param('~linear_reverse', linear_reverse)
    rospy.get_param('~linear_clamp', linear_clamp)
    rospy.get_param('~linear_multiplier', linear_multiplier)

    rospy.get_param('~deadman_button', deadman_button)
    rospy.get_param('~twist_topic', twist_topic)

    rospy.Subscriber("/joy", Joy, joyCallback)
    twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=10)

    rospy.spin()
