#!/usr/bin/env python

# sim harness for trivial baseline control policy. checks forward, left and right sonars and if
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.

import rospy
import argparse
from geometry_msgs.msg import Twist
import sys
from sonars import Sonars
from odom_reward import OdomReward
from collections import OrderedDict
from baseline_policy import BaselinePolicy
from q_table_policy import DiscreteDistanceOrderingQTablePolicy
import json
import util

parser = argparse.ArgumentParser()
parser.add_argument('--robot-id', type=int, default=0)
parser.add_argument('--max-episode-len', type=int, default=1000)
parser.add_argument('--num-episodes', type=int, default=3)
opts = parser.parse_args()

print opts
# helper for max-distance of sonars
sonars = Sonars(opts.robot_id)

# helper for tracking reward based on robot odom
odom_reward = OdomReward(opts.robot_id)

# simple dicrete move-forward, turn-left, turn-right control set
forward = Twist()
forward.linear.x = 1.0
turn_left = Twist()
turn_left.angular.z = 0.6
turn_right = Twist()
turn_right.angular.z = -0.6
steering = rospy.Publisher("/robot%s/cmd_vel" % opts.robot_id, Twist, queue_size=5, latch=True)

rospy.init_node('drivebot_sim')

policy = DiscreteDistanceOrderingQTablePolicy()
#policy = BaselinePolicy()

for episode_id in range(opts.num_episodes):
    # reset robot state
    util.reset_robot(opts.robot_id)
    sonars.reset()
    odom_reward.reset()

    # an episode is a stream of [s1, a, r, s2] events
    # for a async simulated time system the "gap" between a and r is represented by a 'rate' limited loop
    last_state = None
    last_action = None
    episode = []
    rate = rospy.Rate(5)  # hz
    event_id = 0
    last_reward = None
    no_rewards_run_len = 0  # we keep track of runs of 0 rewards as a way of early stopping episode

    while len(episode) < opts.max_episode_len and no_rewards_run_len < 10:
        if rospy.is_shutdown():
            break

        # fetch reward (for last event)
        reward = odom_reward.reward()   

        # keep track of runs of no rewards
        if last_reward == 0 and reward == 0:
            no_rewards_run_len += 1
        else:
            no_rewards_run_len = 0 
        last_reward = reward

        # fetch state (for both s' of last event and s of this event)
        current_state = sonars.ranges
        
        # decide action and apply to bot
        action = policy.action_given_state(current_state)
        if action == 0:
            steering.publish(forward)
        elif action == 1:
            steering.publish(turn_left)
        elif action == 2:
            steering.publish(turn_right)
        else:
            assert False, action

        # flush a single event to episode and train with it
        if last_state is not None:
            event = OrderedDict()
            event['time'] = str(rospy.Time.now())
            event['episode'] = episode_id
            event['event'] = event_id
            event['state_1'] = list(last_state)  # take copy
            event['action'] = last_action
            event['reward'] = reward
            event['state_2'] = list(current_state)
            print "EVENT\t", event, "\tno_rewards_run_len", no_rewards_run_len
            episode.append(event)
            policy.train(last_state, action, reward, current_state)
            event_id += 1
        last_state = current_state
        last_action = action

        # let sim run...
        rate.sleep()

    print "EPISODE\t", json.dumps(episode)
    policy.end_of_episode()

# shutdown bot
steering.publish(Twist())



