#!/bin/bash -eu

unset PYTHONPATH
source /opt/ros/melodic/setup.bash
source /workspace/py3_ws/devel/setup.bash
roslaunch hsr_perception pcd_to_tf.launch &
roslaunch wrs_detector frcnn_finetuned.launch &

wait
