#!/usr/bin/env python

# sim harness for connecting a bot running in a ROS stdr simulation with a decision policy

import rospy
import argparse
from geometry_msgs.msg import Twist
import sys
from sonars import Sonars
from odom_reward import OdomReward
from collections import OrderedDict
from baseline_policy import BaselinePolicy
from q_table_policy import QTablePolicy
import states
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

#state_idxer = states.FurthestSonarIdxer(3)
#policy = BaselinePolicy()

#state_idxer = states.OrderingStateIdxer(3)
state_idxer = states.HistoryStateIdxer(5, 3)
policy = QTablePolicy(num_states=state_idxer.state_space_size(), 
                      num_actions=3, 
                      state_id_to_string_fn=state_idxer.state_idx_to_string)

# TODO: support multiple bots

for episode_id in range(opts.num_episodes):
    # reset robot state
    util.reset_robot(opts.robot_id)
    sonars.reset()
    odom_reward.reset()
    state_idxer.reset()

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

        # get policy to convert lastest sensor reading to a state idx
        current_state = state_idxer.state_idx_given_new_ranges(sonars.ranges)
        print "sonars.ranges=%s => current_state=%s (%s)" % (sonars.ranges, current_state,\
                                                             state_idxer.state_idx_to_string(current_state))

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
            event['t'] = str(rospy.Time.now())
            event['epi_id'] = episode_id
            event['eve_id'] = event_id
            event['s1'] = last_state
            event['a'] = last_action
            event['r'] = reward
            event['s2'] = current_state
            print "EVENT\t", event, "\tno_rewards_run_len", no_rewards_run_len
            episode.append(event)
            policy.train(last_state, action, reward, current_state)
            event_id += 1
        last_state = current_state
        last_action = action

        if len(episode) % 5 == 0:
            policy.debug_model()

        # let sim run...
        rate.sleep()

    print "EPISODE\t", json.dumps(episode)
    policy.end_of_episode()

# shutdown bot
steering.publish(Twist())



