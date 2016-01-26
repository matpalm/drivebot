import numpy as np
import itertools
import sys
from collections import defaultdict, Counter
import random
import states
import util as u

# trivial baseline control policy. checks forward, left and right sonars and if
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.
class BaselinePolicy(object):

    def action_given_state(self, state):
        # expects FurthestSonarIdxer which will directly emit what
        # this policy chooses as an action
        return state

    def train(self, state_1, action, reward, state_2):
        pass

    def debug_model(self):
        pass

    def end_of_episode(self):
        pass

# qtable policy based on a discrete version of sonar values similar to baseline
# states are defined ordering of F, L & R based on order of sonar distances.
# eg state 'FRL' denotes F sonar is closest, then R and finally L.
# results in 6 discrete states.
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.

class QTablePolicy(object):

    def __init__(self, num_actions, discount=0.9, learning_rate=0.2, explore_prob=0.1):
        self.num_actions = num_actions
        self.q_table = defaultdict(lambda : 10 + np.random.uniform(size=num_actions) * 10)
        # debug stats denoting the frequency at which we've updated the qtable entries
        self.state_train_freq = Counter()
        self.discount = discount
        self.learning_rate = learning_rate
        self.explore_prob = explore_prob

    def action_given_state(self, state):
        # trivial explore / exploit
        if random.random() < self.explore_prob:
            # explore
            action = int(random.random() * self.num_actions)
            print "EXPLORE:", action
            return action
        else:
            # exploit: weighted pic of table
            #action = np.argmax(self.q_table[state])
            raised_state_probs = [v**3 for v in self.q_table[state]]
            normed = u.normalised(raised_state_probs)
            action = u.weighted_choice(normed)
            print "CHOOSE: based on state", state, " q_table row", self.q_table[state], " (normed to", normed, ") => action", action
            return action

    def train(self, state_1, action, reward, state_2):
        current_q_s_a = self.q_table[state_1][action]

        max_possible_return_from_state_2 = np.max(self.q_table[state_2])
        candidate_q_s_a = reward + (self.discount * max_possible_return_from_state_2)  # bellman equation

        updated_q_s_a = ((1.-self.learning_rate) * current_q_s_a) + (self.learning_rate * candidate_q_s_a)
        
        print "TRAIN: state_1", state_1, "action", action, "reward", reward, "state_2", state_2,\
            " ... current_q_s_a", current_q_s_a, "max_possible_return_from_state_2", max_possible_return_from_state_2,\
            "candidate_q_s_a", candidate_q_s_a, "updated_q_s_a",updated_q_s_a
        
        self.q_table[state_1][action] = updated_q_s_a
        self.state_train_freq[state_1] += 1

    def debug_model(self):
        print "DEBUG QTABLE:"
        state_freqs = list(self.state_train_freq.iteritems())
        for state, freq in sorted(state_freqs, key=lambda (s, f): -f):            
            print "\t".join(map(str, [state, freq, self.q_table[state]]))

    def end_of_episode(self):
        print ">>> end of episode stats"
        print "state_train_freq", self.state_train_freq
