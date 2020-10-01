#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

pub = rospy.Publisher('/pub_pf', Float32, queue_size=10)
rospy.init_node('pub_pf')
#Rate on which PF estimate is running
rate = 60
r = rospy.Rate(rate)
while not rospy.is_shutdown():
   pub.publish(rate)
   r.sleep()