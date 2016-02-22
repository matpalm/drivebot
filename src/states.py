# utils for converting a stream of sensor data into a state suitable
# for a policy to use.

import copy
import itertools
import numpy as np

# given sonar readings just trivially return index of furthest away
# { F, L, R }
class FurthestSonar:
    def state_given_new_ranges(self, ranges):
        return [np.argmax(ranges)]

    def reset(self):
        pass

    def state_size(self):
        return 1

# given sonar readings return a state based on the 6 possible orderings.
# { FLR, FRL, LFR, LRF, RFL, RLf }
class OrderingSonars:
    def state_given_new_ranges(self, ranges):
        i_v_sorted_by_v = sorted(enumerate(ranges), key=lambda (i, v): -v)
        just_i = [i for (i, v) in i_v_sorted_by_v]
        return just_i

    def reset(self):
        pass

    def state_size(self):
        return 3

# standardises sonar values based on some (precomputed) mean / std
class StandardisedSonars:
    def __init__(self, mean, std):
        self.mean = mean
        self.std = std

    def state_given_new_ranges(self, ranges):
        return [(v-self.mean)/self.std for v in map(float, ranges)]

    def reset(self):
        pass

    def state_size(self):
        return 3


# wrap another sonar reader and keep track of last history_length entries.
class StateHistory:
    def __init__(self, sonar_to_state, history_length):
        self.history_length = history_length
        self.sonar_to_state = sonar_to_state
        self.reset()

    def reset(self):
        self.state_ = []

    def state_given_new_ranges(self, ranges):
        partial_state = self.sonar_to_state.state_given_new_ranges(ranges)
        if len(self.state_) == 0:
            # for first example just buffer up history #hack
            for _ in range(self.history_length):
                self.state_.append(partial_state)
        else:
            self.state_.append(partial_state)
            self.state_.pop(0)
        return copy.deepcopy(self.state_)

    def state_size(self):
        return self.history_length * 3

