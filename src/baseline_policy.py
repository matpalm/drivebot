#!/usr/bin/env python

# trivial baseline control policy. checks forward, left and right sonars and if
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.

import rospy
from geometry_msgs.msg import Twist
import sys
from sonars import Sonars

robot_id = int(sys.argv[1])

# helper for max-distance of sonars
sonars = Sonars(robot_id)

# simple dicrete move-forward, turn-left, turn-right control set
forward = Twist()
forward.linear.x = 0.2
turn_left = Twist()
turn_left.angular.z = 0.2
turn_right = Twist()
turn_right.angular.z = -0.2
steering = rospy.Publisher("/robot%s/cmd_vel" % robot_id, Twist, queue_size=5, latch=True)

rospy.init_node('baseline_policy')

rate = rospy.Rate(1)  # hz
while not rospy.is_shutdown():
    max_sonar_idx = sonars.max_dist_sonar()
    if max_sonar_idx == 0:
        steering.publish(forward)
    elif max_sonar_idx == 1:
        steering.publish(turn_left)
    else:
        steering.publish(turn_right)

    print rospy.Time.now(), sonars.ranges, max_sonar_idx

    rate.sleep()

