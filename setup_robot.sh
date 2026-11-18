#!/bin/bash
read -rp "Enter robot name: " roboName
#In a terminal, ssh into turtlebot of choice
gnome-terminal --tab -- bash -c '
eval "ssh student@${1}.cs.nor.ou.edu"
exec bash
' bash "$roboName"
#In a 2nd terminal, run the robot-setup and build the project.
gnome-terminal --tab -- bash -c '
set -e
read -rp "(2ND) enter anything once the last terminal finishes its job: " check
GEN_COMMANDS=$(/bin/bash robot-setup.sh "${1}" <<< "${1}" | grep -v "Run the following in order:")


eval "$GEN_COMMANDS"

eval "colcon build --symlink-install"
exec bash
' bash "$roboName"

#In a 3rd terminal, run the robot-setup, source venv and setup.bash, export python overload and run
gnome-terminal --tab -- bash -c '
set -e
read -rp "(3RD) enter anything once the last terminal finishes its job: " check
GEN_COMMANDS=$(/bin/bash robot-setup.sh "${1}" <<< "${1}" | grep -v "Run the following in order:")


eval "$GEN_COMMANDS"

source install/setup.bash
cd ..
source venv/bin/activate
cd computer_vision_test
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/$USER/CV_Test/venv/lib/python3.12/site-packages
ros2 run computer_vision object_detection
exec bash
' bash "$roboName"
#In a 4th terminal, run the visualization tool. Make sure to select Yolo topic.
gnome-terminal --tab -- bash -c '
set -e
read -rp "(4TH) enter anything once the last terminal finishes its job: " check
GEN_COMMANDS=$(/bin/bash robot-setup.sh "${1}" <<< "${1}" | grep -v "Run the following in order:")


eval "$GEN_COMMANDS"

ros2 run rqt_image_view rqt_image_view
exec bash
' bash "$roboName"
