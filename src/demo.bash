#!/bin/bash

gnome-terminal -- bash -c "ros2 run turtlesim turtlesim_node"
gnome-terminal -- bash -c "python3 test.py; exec bash"