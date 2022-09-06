#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback(velocity):
	rospy.loginfo("linear: x:%f  angular: z:%f", velocity.linear.x , velocity.angular.z)

def subscriber():
	rospy.init_node("subsciber", anonymous=True)
	rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)
	rospy.spin()

if __name__=="__main__":
	subscriber()
