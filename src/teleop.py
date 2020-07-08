#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def joystick(joy):
    rospy.loginfo('joystick ' + str(joy))
    twist = Twist()
    twist.linear.x = joy.axes[1]*0.5
    twist.linear.y = joy.axes[0]*0.5
    if joy.buttons[4] == 1:
        twist.angular.z = 0.3
    elif joy.buttons[5] == 1:
        twist.angular.z = -0.3
    pub.publish(twist)

# Intializes everything
def start():
    global pub
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.Subscriber("joy", Joy, joystick)
    # starts the node
    rospy.init_node('robin_teleop')
    rospy.spin()

if __name__ == '__main__':
    start()

