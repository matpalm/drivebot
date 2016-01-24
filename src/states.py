# utils for converting a stream of sensor data into a state suitable
# for a policy to use. 

import itertools
import numpy as np

# trivial 3 states for each of F, L or R being furthest away
class FurthestSonar:
    def __init__(self, s_len):
        self.s_len = s_len

    def reset(self):
        pass

    def state_given_new_ranges(self, ranges):
        return np.argmax(ranges)

# given the latest sonars return a state based on the 6 possible orderings of the
# sensors. maintains no history & just uses the latest.
class OrderingSonars:
    def __init__(self, s_len):
        self.s_len = s_len

    def reset(self):
        pass

    def state_given_new_ranges(self, s):
        assert len(s) == self.s_len
        s_with_i = zip(s, range(self.s_len))
        s_with_i = sorted(s_with_i, key=lambda (v, i): -v)
        just_i = [i for (v, i) in s_with_i]
        return tuple(just_i)

# given a stream of sonars (provided by add) keep track of furthest sonar id for 
# the last 'history_length' elements added & use that for state
class HistoryOfFurthestSonar:
    def __init__(self, history_length, s_len):
        self.history_length = history_length
        self.s_len = s_len  # number of elements in s
        self.reset()

    def reset(self):
        self.state_ = [0] * self.history_length  # warm up

    def state_given_new_ranges(self, s):
        # add highest value to history queue
        assert len(s) == self.s_len
        self.state_.append(np.argmax(s))
        self.state_.pop(0)
        return tuple(self.state_)



