# drivebot

for more general info see [the blog post](http://matpalm.com/blog/drivebot/)

## running ros env

stdr sim

```
roslaunch stdr_launchers server_no_map.launch
```

load map and add three bots...

```
rosservice call /stdr_server/load_static_map "mapFile: '$PWD/maps/track1.yaml'"
rosrun stdr_robot robot_handler add $PWD/maps/pandora_robot.yaml 5 5 0
rosrun stdr_robot robot_handler add $PWD/maps/pandora_robot.yaml 5 5 0
rosrun stdr_robot robot_handler add $PWD/maps/pandora_robot.yaml 5 5 0
```

optionally run gui (for general sanity) and rviz (for odom visulations)

```
roslaunch stdr_gui stdr_gui.launch
roslaunch stdr_launchers rviz.launch
```

## running sim

```
$ ./sim.py --help
usage: sim.py [-h] [--robot-id ROBOT_ID] [--max-episode-len MAX_EPISODE_LEN]
              [--num-episodes NUM_EPISODES]
              [--episode-log-file EPISODE_LOG_FILE]
              [--max-no-rewards-run-len MAX_NO_REWARDS_RUN_LEN]
              [--q-discount Q_DISCOUNT] [--q-learning-rate Q_LEARNING_RATE]
              [--q-state-normalisation-squash Q_STATE_NORMALISATION_SQUASH]
              [--sonar-to-state SONAR_TO_STATE]
              [--state-history-length STATE_HISTORY_LENGTH] [--policy POLICY]
              [--gradient-clip GRADIENT_CLIP]
              [--summary-log-dir SUMMARY_LOG_DIR]
              [--summary-log-freq SUMMARY_LOG_FREQ]
              [--target-network-update-freq TARGET_NETWORK_UPDATE_FREQ]
              [--target-network-update-coeff TARGET_NETWORK_UPDATE_COEFF]

optional arguments:
  -h, --help            show this help message and exit
  --robot-id ROBOT_ID
  --max-episode-len MAX_EPISODE_LEN
  --num-episodes NUM_EPISODES
  --episode-log-file EPISODE_LOG_FILE
                        where to write episode log jsonl
  --max-no-rewards-run-len MAX_NO_REWARDS_RUN_LEN
                        early stop episode if no +ve reward in this many steps
  --q-discount Q_DISCOUNT
                        q table discount. 0 => ignore future possible rewards,
                        1 => assume q future rewards perfect. only applicable
                        for QTablePolicies.
  --q-learning-rate Q_LEARNING_RATE
                        q table learning rate. different interp between
                        discrete & nn policies
  --q-state-normalisation-squash Q_STATE_NORMALISATION_SQUASH
                        what power to raise sonar ranges to before
                        normalisation. <1 => explore (tends to uniform), >1 =>
                        exploit (tends to argmax). only applicable for
                        QTablePolicies.
  --sonar-to-state SONAR_TO_STATE
                        what state tranformer to use; FurthestSonar /
                        OrderingSonars / StandardisedSonars
  --state-history-length STATE_HISTORY_LENGTH
                        if >1 wrap sonar-to-state in a StateHistory
  --policy POLICY       what policy to use; Baseline / DiscreteQTablePolicy /
                        NNQTablePolicy
  --gradient-clip GRADIENT_CLIP
  --summary-log-dir SUMMARY_LOG_DIR
                        where to write tensorflow summaries (for the
                        tensorflow models)
  --summary-log-freq SUMMARY_LOG_FREQ
                        freq (in training examples) in which to write to
                        summary
  --target-network-update-freq TARGET_NETWORK_UPDATE_FREQ
                        freq (in training examples) in which to flush core
                        network to target network
  --target-network-update-coeff TARGET_NETWORK_UPDATE_COEFF
                        affine coeff for target network update. 0 => no
                        update, 0.5 => mean of core/target, 1.0 => clobber
                        target completely
```

common config includes 

### baseline

just go in direction of furthest sonar.

state is simply which sonar reads the furthest distance (0 for F, 1 for L and 2 for R) e.g. [0]

```
./sim.py --sonar-to-state FurthestSonar --policy Baseline
```

![lap](blogish/lap_and_half_simple_policy.png?raw=true "lap")

### discrete state q table

q table with discrete states based on which sonar is reporting the furthest distance.

state is concat of history of last 4 readings; eg [0, 0, 1, 1]

```
./sim.py --sonar-to-state FurthestSonar --state-history-length 4 --policy DiscreteQTablePolicy
```

### continous state q learnt function

train a single layer NN using standardised sonar readings (mu and stddev derived from sample data for `discrete state q table`

state is history length * 3 (for F, L & R) tensor; e.g. for history length 2 [[0.0, -0.2, 1.0], [0.3, 0.1, -0.1]]

```
./sim.py --sonar-to-state=StandardisedSonars --state-history-length=4 --policy=NNQTablePolicy
```

## cookbook

### training config

some config is expressed as ros parameters. these params are refreshed per episode by sim / trainer..

```
$ rosparam get /q_table_policy
{ discount: 0.9,                    # bellman RHS discount
  learning_rate: 0.1,               # sgd learning rate
  state_normalisation_squash: 1.0,  # action choosing squash; <1.0 (~0.01) => random choice, >1 (~50) => arg max
  summary_log_freq: 100,            # frequency to write tensorboard summaries
  target_network_update_freq: 10}   # frequency to clobber target network params
```

### episode / event hacking for replay

when `--episode-log-file` is not specified to `sim.py` we can extract it from recorded stdout.

```
$ cat runs/foo.stdout | ./log_to_episodes.py
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
one policy (say a discrete q table) to be used by another policy (say an nn q table). 


```
# rewrite an sequence of episodes so that events retain history of 10 states.
cat runs/foo.log \
 | ./log_to_episodes.py \
 | ./rewrite_event_states.py 10 \
 > runs/foo_episodes_with_new_states.episode.jsonl
```

this data can then be piped through `./episode_to_events.py` and shuffled to build batch experience replay training data.

e.g. replay a sequence of events (one per line) to the `/drivebot/training_egs` ros topic

```
# replay events in random order
cat runs/foo.episode.jsonl | ./episode_to_events.py | shuf | ./publish_events_to_topic.py --rate 1000
```

```
# rewrite episodes to have a history length of 5 then replay events in random order ad infinitum
cat runs/foo.episode.jsonl | ./log_to_episodes.py | ./episode_to_events.py > events.jsonl
while true; do shuf events.jsonl | ./publish_events_to_topic.py --rate=1000; done
```

### stats

```
cat runs/foo.episode.jsonl | ./episode_stats.py > episode_id.num_events.total_reward.tsv
```






