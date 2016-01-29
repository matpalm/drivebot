#!/usr/bin/env python

# sim harness for connecting a bot running in a ROS stdr simulation with a decision policy

import rospy
import argparse
from geometry_msgs.msg import Twist
import sys
from sonars import Sonars
import odom_reward
from collections import OrderedDict
import policy.baseline
import policy.discrete_q_table
import states
import json
import reset_robot_pos

parser = argparse.ArgumentParser()
parser.add_argument('--robot-id', type=int, default=0)
parser.add_argument('--max-episode-len', type=int, default=1000)
parser.add_argument('--num-episodes', type=int, default=100000)
parser.add_argument('--max-no-rewards-run-len', type=int, default=30, help="early stop episode if no +ve reward in this many steps")
parser.add_argument('--state-history-length', type=int, default=4, help="for states.StateHistory")
parser.add_argument('--q-discount', type=float, default=0.9, 
                    help="q table discount. 0 => ignore future possible rewards, 1 => assume q future rewards perfect")
parser.add_argument('--q-learning-rate', type=float, default=0.1, 
                    help="q table learning rate. 0 => never update, 1 => clobber old values completely.")
parser.add_argument('--q-state-normalisation-squash', type=float, default=1.0, 
                    help="what power to raise sonar ranges to before normalisation. <1 => explore (tends to uniform), >1 => exploit (tends to argmax)")
opts = parser.parse_args()

# push args to param server (clumsy, todo: move into qtable)
rospy.set_param("/q_table_policy/discount", opts.q_discount)
rospy.set_param("/q_table_policy/learning_rate", opts.q_learning_rate)
rospy.set_param("/q_table_policy/state_normalisation_squash", opts.q_state_normalisation_squash)

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

rospy.init_node('drivebot_sim')

#sonar_to_state = states.FurthestSonar()
#policy = policy.baseline.BaselinePolicy()

#sonar_to_state = states.OrderingSonars(3)
#sonar_to_state = states.StateHistory(states.OrderingSonars(), opts.state_history_length)
sonar_to_state = states.StateHistory(states.FurthestSonar(), opts.state_history_length)
policy = policy.discrete_q_table.DiscreteQTablePolicy(num_actions=3)

# TODO: support multiple bots. not trivial though since this process, by virtue or rospy.Rate
# is inherently running only one bot...

reset_pos = reset_robot_pos.BotPosition(opts.robot_id)

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
            event_id += 1

        # flush last_XYZ for next event
        last_ranges = current_ranges
        last_state = current_state
        last_action = action

        # let sim run...
        rate.sleep()

    print "EPISODE\t", json.dumps(episode)
    policy.end_of_episode()

# shutdown bot
steering.publish(Twist())



