#!/usr/bin/env python
import yaml
import random
import math

NUM_SONARS = 5

d = {"robot": {"robot_specifications": []}}
rs = d["robot"]["robot_specifications"]
rs.append({"footprint": 
             {"footprint_specifications": 
                {"radius": 0.2, "points": []}}})
rs.append({"initial_pose": 
             {"x": 0, "y": 0, "theta": 0}})

def sonar(sid, theta):
    return {"sonar_specifications": 
              {"cone_angle": 0.87, "max_range": 3, "min_range": 0.15,
                "frequency": 10, "frame_id": ("sonar_%d" % sid),
                "pose": {"x": 0, "y": 0, "theta": theta},
                "noise": 
                  {"noise_specifications": 
                    {"noise_mean": 0.1, "noise_std": 0.01 }}}}

for sonar_id in xrange(5):
    theta = random.random() * 2 * math.pi
    rs.append({"sonar": sonar(sonar_id, theta)})

import yaml
print yaml.dump(d, default_flow_style=False)


