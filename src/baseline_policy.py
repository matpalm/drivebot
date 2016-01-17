#!/usr/bin/env python

# trivial baseline control policy. checks forward, left and right sonars and if
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.

import rospy
from geometry_msgs.msg import Twist
import sys
from sonars import Sonars
from odom_reward import OdomReward
from collections import OrderedDict
import json

robot_id = int(sys.argv[1])

# helper for max-distance of sonars
sonars = Sonars(robot_id)

# helper for tracking reward based on robot odom
odom_reward = OdomReward(robot_id)

# simple dicrete move-forward, turn-left, turn-right control set
forward = Twist()
forward.linear.x = 0.4
turn_left = Twist()
turn_left.angular.z = 0.3
turn_right = Twist()
turn_right.angular.z = -0.3
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

    event = OrderedDict()
    event["time"] = str(rospy.Time.now())
    event["ranges"] = sonars.ranges
    event["action"] = max_sonar_idx
    event["reward"] = odom_reward.reward()
    print json.dumps(event)

    rate.sleep()

