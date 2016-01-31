# drivebot

## running ros env

stdr sim & gui with single bot

```
roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch 
```

optionally run rviz at same time (eg for odom visulations)

```
roslaunch stdr_launchers rviz.launch
```

## running sim

for now hack code for changing policy. supports baseline hard coded rules vs simple q table
learning with a couple of different representations of state (eg ordering of sonars vs
history of furthest sonar)

![lap](blogish/lap_and_half_simple_policy.png?raw=true "lap")

### cookbook

extract just episode jsonl from a log (one episode per line)

```
$ cat runs/foo.log | ./log_to_episodes.py
```

extract event jsonl from a log (one event per line)

```
$ cat runs/foo.log | ./log_to_episodes.py | ./episode_to_events.py
```

use previous date to calculate mean/std of ranges

```
$ cat runs/foo.log | ./log_to_episodes.py | ./calculate_range_standardisation.py
(68.56658770018004, 30.238156000781927)
```

rewrite the states for a run. every simualtion involves mapping from ranges -> states that are specific
for the policy being trained. by using `./rewrite_event_states.py` we can rewrite the ranges (which are always the
same) to be a different state sequence. this can be used to build cirruculum style training data from
one policy (say a discrete q table) to be used by another policy (say an nn q table). hack `./rewrite_event_states.py`
directly to describe the form of the desired output state.

```
cat runs/foo.log \
 | ./log_to_episodes.py \
 | ./rewrite_event_states.py \
 > runs/foo_with_new_states.log
```

this data can then be piped through `./episode_to_events.py` and shuffled to build batch experience replay training data.





