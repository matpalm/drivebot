#!/usr/bin/env python
import rospy
from sonars import Sonars
import states
import sys
s = Sonars(int(sys.argv[1]))
ss = states.StandardisedSonars(mean=59.317, std=37.603)
rospy.init_node("test_sonars")
rate = rospy.Rate(5)  # hz 
while not rospy.is_shutdown():
    r = s.ranges
    print r, ss.state_given_new_ranges(r)
    rate.sleep()

