#!/usr/bin/env python

# qtable policy based on a discrete version of sonar values similar to baseline
# states are defined ordering of F, L & R based on order of sonar distances.
# eg state 'FRL' denotes F sonar is closest, then R and finally L.
# results in 6 discrete states.
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.

from sonars import Sonars
import numpy as np
import itertools
import sys
from collections import Counter
import random
import states

DISCOUNT = 0.9  # theta
LEARNING_RATE = 0.2 # alpha
EXPLORE_PROB = 0.1

# TODO: make qtable sparse; ie make rows on demand
# TODO: make output of qtable loggable (QTABLE\tJSON) for ease of processing stats
# TODO: make threadsafe so sim can run 2+ bots

class QTablePolicy(object):

    def __init__(self, num_states, num_actions, state_id_to_string_fn):
        self.num_states = num_states
        self.num_actions = num_actions
        self.q_table = 10 + np.random.uniform(size=(num_states, num_actions)) * 10
        # debug stats denoting the frequency at which we've updated the qtable rows
        self.state_train_freq = Counter()
        self.state_id_to_string = state_id_to_string_fn

    def action_given_state(self, state):
        # trivial explore / exploit
        if random.random() < EXPLORE_PROB:
            # explore
            action = int(random.random() * self.num_actions)
            print "EXPLORE", action
            return action
        else:
            # exploit: arg max of this table
            action = np.argmax(self.q_table[state])
            print "CHOOSE: from state", state, "(", self.state_id_to_string(state), ") q_table row", self.q_table[state], " action", action
            return action

    def train(self, state_1, action, reward, state_2):
        current_q_s_a = self.q_table[state_1][action]

        max_possible_return_from_state_2 = np.max(self.q_table[state_2])
        candidate_q_s_a = reward + (DISCOUNT * max_possible_return_from_state_2)  # bellman equation

        updated_q_s_a = ((1.0 - LEARNING_RATE) * current_q_s_a) + (LEARNING_RATE * candidate_q_s_a)
        
        print "TRAIN: state_1", state_1, "(", self.state_id_to_string(state_1), ")",\
            "action", action, "reward", reward,\
            "state_2", state_2, "(", self.state_id_to_string(state_2), ")",\
            " ... current_q_s_a", current_q_s_a, "max_possible_return_from_state_2", max_possible_return_from_state_2,\
            "candidate_q_s_a", candidate_q_s_a, "updated_q_s_a",updated_q_s_a
        
        self.q_table[state_1][action] = updated_q_s_a
        self.state_train_freq[state_1] += 1

    def debug_model(self):
        print "DEBUG QTABLE"
        for i in range(len(self.q_table)):
            if self.state_train_freq[i] > 0:
                print i, "\t", self.state_id_to_string(i), "\t", self.state_train_freq[i], "\t", self.q_table[i]

    def end_of_episode(self):
        print ">>> end of episode stats"
        print "state_train_freq", self.state_train_freq
        print "<<< end of episode stats"
