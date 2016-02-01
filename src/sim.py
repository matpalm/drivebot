#!/usr/bin/env python

# sim harness for connecting a bot running in a ROS stdr simulation with a decision policy

import argparse
from collections import OrderedDict
from drivebot.msg import TrainingExample
from geometry_msgs.msg import Twist
import json
import math
import numpy as np
import odom_reward
import policy.baseline
import policy.discrete_q_table
import policy.nn_q_table
import reset_robot_pos
import rospy
from sonars import Sonars
import states
import sys
import util as u

parser = argparse.ArgumentParser()
parser.add_argument('--robot-id', type=int, default=0)
parser.add_argument('--max-episode-len', type=int, default=1000)
parser.add_argument('--num-episodes', type=int, default=100000)
parser.add_argument('--episode-log-file', default="/dev/stdout", help="where to write episode log jsonl")
parser.add_argument('--max-no-rewards-run-len', type=int, default=30, help="early stop episode if no +ve reward in this many steps")
parser.add_argument('--q-discount', type=float, default=0.9, 
                    help="q table discount. 0 => ignore future possible rewards, 1 => assume q future rewards perfect. only applicable for QTablePolicies.")
parser.add_argument('--q-learning-rate', type=float, default=0.1, 
                    help="q table learning rate. 0 => never update, 1 => clobber old values completely. only applicable for QTablePolicies.")
parser.add_argument('--q-state-normalisation-squash', type=float, default=0.001, 
                    help="what power to raise sonar ranges to before normalisation."\
                         " <1 => explore (tends to uniform), >1 => exploit (tends to argmax)."\
                         " only applicable for QTablePolicies.")
parser.add_argument('--sonar-to-state', type=str, default="FurthestSonar",
                    help="what state tranformer to use; FurthestSonar / OrderingSonars / StandardisedSonars")
parser.add_argument('--state-history-length', type=int, default=0, help="if >1 wrap sonar-to-state in a StateHistory")
parser.add_argument('--policy', type=str, default="Baseline",
                    help="what policy to use; Baseline / DiscreteQTablePolicy / NNQTablePolicy")
parser.add_argument('--summary-log-dir', type=str, default="/tmp/nn_q_table",
                    help="where to write tensorflow summaries (for the tensorflow models)")
opts = parser.parse_args()
print "OPTS", opts

episode_log = open(opts.episode_log_file, "w")

# push args to param server (clumsy, todo: move into qtable)
rospy.set_param("/q_table_policy/discount", opts.q_discount)
rospy.set_param("/q_table_policy/learning_rate", opts.q_learning_rate)
rospy.set_param("/q_table_policy/state_normalisation_squash", opts.q_state_normalisation_squash)

# config sonar -> state transformation
if opts.sonar_to_state == "FurthestSonar":
    sonar_to_state = states.FurthestSonar()
elif opts.sonar_to_state == "OrderingSonars":
    sonar_to_state = states.OrderingSonars()
elif opts.sonar_to_state == "StandardisedSonars":
    sonar_to_state = states.StandardisedSonars(mean=59.317, std=37.603)
else:
    raise Exception("unknown --sonar-to-state %s" % opts.sonar_to_state)
if opts.state_history_length > 1:
    sonar_to_state = states.StateHistory(sonar_to_state, opts.state_history_length)

# config state -> action policy
if opts.policy == "Baseline":
    policy = policy.baseline.BaselinePolicy()
elif opts.policy == "DiscreteQTablePolicy":
    policy = policy.discrete_q_table.DiscreteQTablePolicy(num_actions=3)
elif opts.policy == "NNQTablePolicy":
    hidden_size = int(math.sqrt(sonar_to_state.state_size() * 3))
    print "NNQTablePolicy #input", sonar_to_state.state_size(), "#hidden", hidden_size
    policy = policy.nn_q_table.NNQTablePolicy(state_size=sonar_to_state.state_size(),
                                              num_actions=3, hidden_layer_size=hidden_size,
                                              summary_file=opts.summary_log_dir)
else:
    raise Exception("unknown --policy %s" % opts.policy)

# helper for max-distance of sonars
sonars = Sonars(opts.robot_id)

# helper for tracking reward based on robot odom
#odom_reward = odom_reward.CoarseGridReward(opts.robot_id)
odom_reward = odom_reward.MovingOdomReward(opts.robot_id)

# simple dicrete move-forward, turn-left, turn-right control set
forward = Twist()
forward.linear.x = 1.0
turn_left = Twist()
turn_left.angular.z = 1.2
turn_right = Twist()
turn_right.angular.z = -1.2
steering = rospy.Publisher("/robot%s/cmd_vel" % opts.robot_id, Twist, queue_size=5, latch=True)

# publish training events
training = rospy.Publisher("/drivebot/training_egs", TrainingExample, queue_size=200)

# init ros node
rospy.init_node('drivebot_sim')
reset_pos = reset_robot_pos.BotPosition(opts.robot_id)

# run sim for awhile
for episode_id in range(opts.num_episodes):
    # reset robot state
    reset_pos.reset_robot_random_pose()  # totally random pose
    #reset_pos.reset_robot_on_straight_section()  # reset on track, on straight section, facing clockwise
    sonars.reset()
    odom_reward.reset()
    sonar_to_state.reset()

    # an episode is a stream of [state_1, action, reward, state_2] events
    # for a async simulated time system the "gap" between a and r is represented by a 'rate' limited loop
    # for debugging (and more flexible replay) we also keep track of the raw sonar.ranges in the event
    last_ranges = None
    last_state = None
    last_action = None
    episode = []
    rate = rospy.Rate(5)  # hz
    event_id = 0
    last_reward = None
    no_rewards_run_len = 0  # we keep track of runs of 0 rewards as a way of early stopping episode

    while len(episode) < opts.max_episode_len and no_rewards_run_len < opts.max_no_rewards_run_len:
        if rospy.is_shutdown():
            break

        # fetch reward (for last event)
        reward = odom_reward.reward(last_action)

        # keep track of runs of no +ve reward
        if last_reward <= 0 and reward <= 0:
            no_rewards_run_len += 1
        else:
            no_rewards_run_len = 0 
        last_reward = reward

        # if last_action was move forward, and we got no reward from it, punish thins

        # get policy to convert lastest sensor reading to a state idx
        current_ranges = list(sonars.ranges)
        current_state = sonar_to_state.state_given_new_ranges(current_ranges)

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
            event['epi_id'] = episode_id
            event['eve_id'] = event_id
            event['ranges_1'] = last_ranges
            event['state_1'] = last_state
            event['action'] = last_action
            event['reward'] = reward
            event['ranges_2'] = current_ranges
            event['state_2'] = current_state
            print "EVENT\tepi_id=%s eve_id=%s no_rewards_run_len=%s" % (episode_id, event_id, no_rewards_run_len)
            episode.append(event)
            policy.train(last_state, last_action, reward, current_state)
            # move to this eventually when qtable is running out of policy
            # training.publish(u.training_eg_msg(last_state, last_action, reward, current_state))  
            event_id += 1

        # flush last_XYZ for next event
        last_ranges = current_ranges
        last_state = current_state
        last_action = action

        # let sim run...
        rate.sleep()

    # write episode to log
    print >>episode_log, json.dumps(episode)
    episode_log.flush()

    policy.end_of_episode()

steering.publish(Twist())  # shutdown movement of bot

episode_log.close()



