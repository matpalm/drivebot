# utils for converting a stream of sensor data into a state suitable
# for a policy to use. 

import itertools
import numpy as np

# trivial 3 state machine for each of F, L or R being furthest away
class FurthestSonarIdxer:
    def __init__(self, s_len):
        self.s_len = s_len

    def reset(self):
        pass

    def state_space_size(self):
        return self.s_len

    # add ranges and return state idx
    def state_idx_given_new_ranges(self, ranges):
        return np.argmax(ranges)
       
    def state_idx_to_string(self, state):
        return state

# given the latest state return a state_idx based on the 6 ordering of the
# sensors. maintains no history & just uses the latest.
class OrderingStateIdxer:
    def __init__(self, s_len):
        self.s_len = s_len
        range_perms = list(enumerate(itertools.permutations(range(self.s_len))))
        self.perm_idx_to_state_idx = {v: k for (k, v) in range_perms}
        self.state_idx_to_perm_idx = {k: v for (k, v) in range_perms}
        print "state_idx_to_perm_idx", self.state_idx_to_perm_idx
        
    def reset(self):
        pass

    def state_space_size(self):
        return len(self.perm_idx_to_state_idx)

    def state_idx_given_new_ranges(self, s):
        assert len(s) == self.s_len
        s_with_i = zip(s, range(self.s_len))
        s_with_i = sorted(s_with_i, key=lambda (v, i): -v)
        just_i = [i for (v, i) in s_with_i]
        return self.perm_idx_to_state_idx[tuple(just_i)]

    def state_idx_to_string(self, state):
        return self.state_idx_to_perm_idx[state]

# given a stream of s's (provided by add) keep track of furthest sonar id for 
# the last 'history_length' elements added & use that for state_idx
class HistoryStateIdxer:
    def __init__(self, history_length, s_len):
        self.history_length = history_length
        self.s_len = s_len  # number of elements in s
        self.reset()

    def reset(self):
        self.state_ = [0] * self.history_length  # warm up

    def state_space_size(self):
        return self.s_len ** self.history_length

    def state_idx_given_new_ranges(self, s):
        # add highest value to history queue
        assert len(s) == self.s_len
        self.state_.append(np.argmax(s))
        self.state_.pop(0)
        # pack history into base s_len number and return
        # ergh :/ convert to string (?) ugly...
        return int("".join(map(str, self.state_)), base=self.s_len)

    def state_idx_to_string(self, state):
        # return base s_len version of base 10 value 'state'
        return np.base_repr(state, base=self.s_len).zfill(self.history_length)


