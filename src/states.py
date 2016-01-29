# utils for converting a stream of sensor data into a state suitable
# for a policy to use.

import itertools
import numpy as np

# given sonar readings just trivially return index of furthest away
# { F, L, R }
class FurthestSonar:
    def state_given_new_ranges(self, ranges):
        return np.argmax(ranges)

    def reset(self):
        pass

# given sonar readings return a state based on the 6 possible orderings.
# { FLR, FRL, LFR, LRF, RFL, RLf }
class OrderingSonars:
    def state_given_new_ranges(self, s):
        i_v_sorted_by_v = sorted(enumerate(s), key=lambda (i, v): -v)
        just_i = [i for (i, v) in i_v_sorted_by_v]
        return tuple(just_i)

    def reset(self):
        pass

# wrap another sonar reader and keep track of last history_length entries.
class StateHistory:
    def __init__(self, sonar_to_state, history_length):
        self.history_length = history_length
        self.sonar_to_state = sonar_to_state
        self.reset()

    def reset(self):
        self.state_ = []

    def state_given_new_ranges(self, s):
        self.state_.append(self.sonar_to_state.state_given_new_ranges(s))
        if len(self.state_) > self.history_length:
            self.state_.pop(0)
        return tuple(self.state_)


