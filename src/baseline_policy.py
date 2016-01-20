#!/usr/bin/env python

# trivial baseline control policy. checks forward, left and right sonars and if
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.

from sonars import Sonars, max_dist_sonar

class BaselinePolicy(object):

    def action_given_state(self, ranges):
        # action is just index of max distance sonar
        return max_dist_sonar(ranges)

    def train(self, state_1, action, reward, state_2):
        pass

    def end_of_episode(self):
        pass


