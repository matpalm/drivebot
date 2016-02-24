import rospy
import numpy as np
import itertools
import sys
from collections import defaultdict, Counter
import random
import states
import util as u

# qtable policy based on a discrete version of sonar values similar to baseline
# states are defined ordering of F, L & R based on order of sonar distances.
# eg state 'FRL' denotes F sonar is closest, then R and finally L.
# results in 6 discrete states.
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.

class DiscreteQTablePolicy(object):

    def __init__(self, num_actions):
        self.num_actions = num_actions
        self.q_table = defaultdict(lambda : 10 + np.random.uniform(size=num_actions) * 3)
        # debug stats denoting the frequency at which we've updated the qtable entries
        self.state_train_freq = Counter()
        self.refresh_params()
        self.calls_to_train = 0

    def refresh_params(self):
        params = rospy.get_param("q_table_policy")
        print "REFRESH_PARAM\t%s" % params
        self.discount = params['discount']
        self.learning_rate = params['learning_rate']
        self.state_normalisation_squash = params['state_normalisation_squash']

    def q_values_normalised_for_pick(self, state):
        return u.normalised(u.raised(self.q_table[state], self.state_normalisation_squash))
        
    def action_given_state(self, state):
        if type(state) == list:
            state = tuple(state)
        normed = self.q_values_normalised_for_pick(state)
        action = u.weighted_choice(normed)
        print "CHOOSE\t based on state", state, "q_values", self.q_table[state], "(normed to", normed, ") => action", action
        return action

    def train(self, state_1, action, reward, state_2):
        if type(state_1) == list:
            state_1, state_2 = tuple(state_1), tuple(state_2)

        # update q table
        current_q_s_a = self.q_table[state_1][action]
        max_possible_return_from_state_2 = np.max(self.q_table[state_2])
        candidate_q_s_a = reward + (self.discount * max_possible_return_from_state_2)  # bellman equation
        updated_q_s_a = ((1.-self.learning_rate) * current_q_s_a) + (self.learning_rate * candidate_q_s_a)
        self.q_table[state_1][action] = updated_q_s_a

        # debug
        print "TRAIN\tstate_1", state_1, "action", action, "reward", reward, "state_2", state_2,\
            " ... current_q_s_a", current_q_s_a, "max_possible_return_from_state_2", max_possible_return_from_state_2,\
            "candidate_q_s_a", candidate_q_s_a, "updated_q_s_a",updated_q_s_a        

        # stats
        self.state_train_freq[state_1] += 1
        self.calls_to_train += 1
        if self.calls_to_train % 100 == 0:
            print ">>> end of episode stats"
            print "DEBUG QTABLE:"
            state_freqs = list(self.state_train_freq.iteritems())
            for state, freq in sorted(state_freqs, key=lambda (s, f): -f):
                if freq > 2:
                    print "\t".join(map(str, [state, freq, self.q_table[state], self.q_values_normalised_for_pick(state)]))
            print "state_train_freq", self.state_train_freq
            self.refresh_params()
