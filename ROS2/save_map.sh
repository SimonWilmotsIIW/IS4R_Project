#!/bin/bash

ros2 run nav2_map_server map_saver_cli -t map -f ./ros_packages/turtlebot3/resource/simons_new_map --occ 0.65 --free 0.25 --mode trinary --fmt pgm \
#&& ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
