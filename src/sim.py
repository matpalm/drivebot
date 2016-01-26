#!/usr/bin/env python

# sim harness for connecting a bot running in a ROS stdr simulation with a decision policy

import rospy
import argparse
from geometry_msgs.msg import Twist
import sys
from sonars import Sonars
import odom_reward
from collections import OrderedDict
import policy
import states
import json
import reset_robot_pos

parser = argparse.ArgumentParser()
parser.add_argument('--robot-id', type=int, default=0)
parser.add_argument('--max-episode-len', type=int, default=1000)
parser.add_argument('--num-episodes', type=int, default=3)
parser.add_argument('--max-no-rewards-run-len', type=int, default=30)
parser.add_argument('--state-history-length', type=int, default=5, help="for HistoryOfFurthestSonar")
parser.add_argument('--q-discount', type=float, default=0.9, 
                    help="q table discount. 0 => ignore future possible rewards, 1 => assume q future rewards perfect")
parser.add_argument('--q-learning-rate', type=float, default=0.1, 
                    help="q table learning rate. 0 => never update, 1 => clobber old values completely.")
parser.add_argument('--q-explore-prob', type=float, default=0.1, 
                    help="trivial explore prob (vs exploit by picking arg max Q)")
opts = parser.parse_args()
print opts

# helper for max-distance of sonars
sonars = Sonars(opts.robot_id)

# helper for tracking reward based on robot odom
#odom_reward = odom_reward.CoarseGridReward(opts.robot_id)
odom_reward = odom_reward.MovingOdomReward(opts.robot_id)

# simple dicrete move-forward, turn-left, turn-right control set
forward = Twist()
forward.linear.x = 1.0
turn_left = Twist()
turn_left.angular.z = 0.6
turn_right = Twist()
turn_right.angular.z = -0.6
steering = rospy.Publisher("/robot%s/cmd_vel" % opts.robot_id, Twist, queue_size=5, latch=True)

rospy.init_node('drivebot_sim')

#sonar_to_state = states.FurthestSonar(3)
#policy = policy.BaselinePolicy()

#sonar_to_state = states.OrderingSonars(3)
sonar_to_state = states.HistoryOfFurthestSonar(opts.state_history_length, 3)
policy = policy.QTablePolicy(num_actions=3, discount=opts.q_discount, 
                             learning_rate=opts.q_learning_rate, 
                             explore_prob=opts.q_explore_prob)

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

    # an episode is a stream of [s1, a, r, s2] events
    # for a async simulated time system the "gap" between a and r is represented by a 'rate' limited loop
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
        reward = odom_reward.reward()   

        # keep track of runs of no rewards
        if last_reward == 0 and reward == 0:
            no_rewards_run_len += 1
        else:
            no_rewards_run_len = 0 
        last_reward = reward

        # get policy to convert lastest sensor reading to a state idx
        current_state = sonar_to_state.state_given_new_ranges(sonars.ranges)
        print "sonars.ranges=%s => current_state=%s" % (sonars.ranges, current_state)

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



