#!/bin/bash

# Cleanup
unset GAZEBO_MODEL_PATH
unset GAZEBO_PLUGIN_PATH
unset GAZEBO_RESOURCE_PATH

# ROS environment setup
export ROS_PACKAGE_PATH=`pwd`:/usr/share/osrf-common-1.0/ros:/opt/ros/fuerte/share:/opt/ros/fuerte/stacks

# Setup basic Gazebo environment
if [ -f /usr/share/gazebo-1.8/setup.sh ]; then
  source /usr/share/gazebo-1.8/setup.sh
else
  source /usr/share/gazebo/setup.sh
fi

# Gazebo - ros_control
export GAZEBO_PLUGIN_PATH=`rospack find ros_control_gazebo_plugin`/lib:$GAZEBO_PLUGIN_PATH # plugins

# Gazebo - atlas_msgs
export GAZEBO_PLUGIN_PATH=`rospack find atlas_msgs`/lib:$GAZEBO_PLUGIN_PATH             # plugins

# Gazebo - PAL
export GAZEBO_PLUGIN_PATH=`rospack find pal_gazebo_plugins`/lib:$GAZEBO_PLUGIN_PATH     # plugins

# Gazebo - REEM-C
export GAZEBO_MODEL_PATH=`rospack find reemc_gazebo`/models:$GAZEBO_MODEL_PATH          # models
export GAZEBO_RESOURCE_PATH=`rospack find reemc_gazebo`:$GAZEBO_RESOURCE_PATH           # resources
