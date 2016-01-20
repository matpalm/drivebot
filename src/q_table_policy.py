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

DISCOUNT = 0.9  # theta
LEARNING_RATE = 0.2 # alpha

class DiscreteDistanceOrderingQTablePolicy(object):

    def __init__(self):
        o, t = 10, 20
        #self.q_table = np.asarray([[t,o,o],[t,o,o],[o,t,o],[o,t,o],[o,o,t],[o,o,t]], dtype='float32')
        self.q_table = 10 + np.random.uniform(size=(6, 3)) * 10 # 6 states (see below) and 3 actions
        print "QTABLE", self.q_table

        # our states will be the 6 combination orderings of 0, 1, 2 corresponding
        # to the sonar idxs for F, L & R respectively. we want a lookup table from
        # these perms into the rows of the q_table
        # TODO pull out of class, no per bot state here...
        perms = itertools.permutations([0, 1, 2], r=3)  # 6 permutations
        self.perm_to_state = {perm: idx for idx, perm in enumerate(perms)}  # {(0,1,2): 0, (0,2,1): 1, ... }
        print "self.perm_to_state", self.perm_to_state

        # debug / stats
        self.state_train_freq = Counter()

    def sonar_values_to_state_idx(self, ranges):
        # map three sonar values to a state_idx representing 1 of 6 combos
        sorted_values_with_idxs = sorted(enumerate(ranges), key=lambda (idx, value): -value)
        just_idxs = tuple([v[0] for v in sorted_values_with_idxs])
        return self.perm_to_state[just_idxs]

    def action_given_state(self, ranges):
        # TODO: explore / exploit
        if random.random() < 0.5:  # HACKTASTIC EXPLORE
            return int(random.random() * 3)

        print ">action_given_state", ranges
        if ranges[0] is None:
            # race condition with sonar, just go forward...
            print >>sys.stderr, "null sonar reading? just startup race?"
            return 0
        state_idx = self.sonar_values_to_state_idx(ranges)
        action = np.argmax(self.q_table[state_idx])
        print "state_idx", state_idx, "(q_table row", self.q_table[state_idx], ") action", action
        return action

    def train(self, state_1, action_idx, reward, state_2):
        state_1_idx = self.sonar_values_to_state_idx(state_1)
        state_2_idx = self.sonar_values_to_state_idx(state_2)

        current_q_s_a = self.q_table[state_1_idx][action_idx]

        max_possible_return_from_state_2 = np.max(self.q_table[state_2_idx])
        candidate_q_s_a = reward + (DISCOUNT * max_possible_return_from_state_2)  # bellman equation

        updated_q_s_a = ((1.0 - LEARNING_RATE) * current_q_s_a) + (LEARNING_RATE * candidate_q_s_a)
        
        print "state_1_idx", state_1_idx, "action_idx", action_idx, "reward", reward, "state_2_idx", state_2_idx,\
            " ... current_q_s_a", current_q_s_a, "candidate_q_s_a", candidate_q_s_a, "updated_q_s_a",updated_q_s_a
        print self.q_table
        self.q_table[state_1_idx][action_idx] = updated_q_s_a
        self.state_train_freq[state_1_idx] += 1

    def end_of_episode(self):
        print ">>> end of episode stats"
        print "state_train_freq", self.state_train_freq
        print "<<< end of episode stats"
