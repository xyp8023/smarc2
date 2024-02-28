#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/colcon_ws/install/setup.bash

exec "$@"