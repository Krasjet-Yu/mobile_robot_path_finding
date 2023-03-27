#!/bin/bash

rostopic pub --once /goal geometry_msgs/PoseStamped "header:
    seq: 0
    stamp:
        secs: 6
        nsecs: 0
    frame_id: 'map'
pose:
    position:
        x: 8
        y: 8
        z: 0.0
    orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0"
