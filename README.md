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