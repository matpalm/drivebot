#!/usr/bin/env python

# trivial baseline control policy. checks forward, left and right sonars and if
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.

import rospy
import argparse
from geometry_msgs.msg import Twist
import sys
from sonars import Sonars
from odom_reward import OdomReward
from collections import OrderedDict
import json
import util

parser = argparse.ArgumentParser()
parser.add_argument('--robot_id', default=0)
parser.add_argument('--episode_len', default=100)
opts = parser.parse_args()

# helper for max-distance of sonars
sonars = Sonars(opts.robot_id)

# helper for tracking reward based on robot odom
odom_reward = OdomReward(opts.robot_id)

# simple dicrete move-forward, turn-left, turn-right control set
forward = Twist()
forward.linear.x = 0.4
turn_left = Twist()
turn_left.angular.z = 0.3
turn_right = Twist()
turn_right.angular.z = -0.3
steering = rospy.Publisher("/robot%s/cmd_vel" % opts.robot_id, Twist, queue_size=5, latch=True)

rospy.init_node('baseline_policy')

util.reset_robot(opts.robot_id)
sonars.reset()
odom_reward.reset()

rate = rospy.Rate(1)  # hz
episode = []
while len(episode) < opts.episode_len:
    if rospy.is_shutdown():
        break

    max_sonar_idx = sonars.max_dist_sonar()
    if max_sonar_idx == 0:
        steering.publish(forward)
    elif max_sonar_idx == 1:
        steering.publish(turn_left)
    else:
        steering.publish(turn_right)

    event = {}
    event['time'] = str(rospy.Time.now())
    event['sonars'] = sonars.ranges
    event['action'] = max_sonar_idx
    event['reward'] = odom_reward.reward()
    episode.append(event)

    rate.sleep()

print json.dumps(episode)

