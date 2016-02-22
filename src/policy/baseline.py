# trivial baseline control policy. checks forward, left and right sonars and if
# forward reports the largest distance move forward and if left/right reports
# largest distance turn (on spot) left/right.
class BaselinePolicy(object):

    def action_given_state(self, state):
        # expects FurthestSonarIdxer which will directly emit what
        # this policy chooses as an action. a bit clumsy, better to
        # type this state somehow...
        if len(state) != 1:
            raise TypeError("expect only single value in state, not %d values" % len(state))
        state = state[0]
        if state not in [0, 1, 2]:
            raise TypeError("expect only 0,1,2 not %d" % state)
        return state

    def train(self, state_1, action, reward, state_2):
        pass

