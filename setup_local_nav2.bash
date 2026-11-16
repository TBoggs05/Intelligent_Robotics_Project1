#!/usr/bin/env bash

project1_nav2_prefix="$HOME/.local/share/project1_ros_overlay/opt/ros/jazzy"
if [[ ! -x "$project1_nav2_prefix/lib/nav2_lifecycle_manager/lifecycle_manager" ]]; then
    printf '%s\n' 'Local Nav2 overlay not found; see the no-administrator setup in README.md.' >&2
    unset project1_nav2_prefix
    return 1
fi

export AMENT_PREFIX_PATH="$project1_nav2_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$project1_nav2_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset project1_nav2_prefix
