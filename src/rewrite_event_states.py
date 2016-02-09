#!/usr/bin/env python
import json, sys
import states

#sonar_to_state = states.FurthestSonar() # OrderingSonars()
#sonar_to_state = states.OrderingSonars()
#sonar_to_state = states.StateHistory(states.FurthestSonar(), history_length=4)
sonar_to_state = states.StateHistory(states.StandardisedSonars(mean=59.317, std=37.603), 
                                     history_length=int(sys.argv[1]))

for line in sys.stdin:
    episode = json.loads(line)
    rewritten_episode = []

    last_state = None
    for event in episode:

        # for the very first even we need to use ranges1 to decide state1
        # but from then on we just copy the last value across
        if last_state is None:
            event['state_1'] = sonar_to_state.state_given_new_ranges(event['ranges_1'])
        else:
            event['state_1'] = last_state

        # update state2
        event['state_2'] = sonar_to_state.state_given_new_ranges(event['ranges_2'])
        last_state = event['state_2']

        rewritten_episode.append(event)

    print json.dumps(rewritten_episode)

