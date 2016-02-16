#!/bin/bash

if [ $# -eq 0 ]
then
    echo "Usage: ikfast.sh <ROBOT-NAME>"
else    

# Reference:
# http://docs.ros.org/indigo/api/moveit_ikfast/html/doc/ikfast_tutorial.html

ROBOT=$1
#bypass: rosrun collada_urdf urdf_to_collada ${ROBOT}.urdf ${ROBOT}.dae
#bypass: rosrun moveit_ikfast round_collada_numbers.py ${ROBOT}.dae ${ROBOT}.rounded.dae 5
#bypass: openrave-robot.py ${ROBOT}.dae --info links
#bypass: python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py \
#bypass:         --robot=${ROBOT}.dae --iktype=transform6d --baselink=0 --eelink=6 \
#bypass:         --savefile=ikfast61_${ROBOT}.cpp

IK_CPP=${PWD}/ikfast61_${ROBOT}.cpp

pushd ~/catkin_ws/src
# @planning_group_name - arm1
PLAN_GROUP=arm1
# @moveit_ik_plugin_pkg - ${ROBOT}_ikfast_${PLAN_GROUP}_plugin
IK_PKG=${ROBOT}_ikfast_${PLAN_GROUP}_plugin
if ! [ -d ${IK_PKG} ]
then 
    catkin_create_pkg ${IK_PKG}
    cd ~/catkin_ws
    catkin_make # add $IK_PKG to search path
fi
echo "rosrun moveit_ikfast create_ikfast_moveit_plugin.py ${ROBOT} ${PLAN_GROUP} ${ROBOT}_ikfast_${PLAN_GROUP}_plugin ${IK_CPP}"
rosrun moveit_ikfast create_ikfast_moveit_plugin.py \
    ${ROBOT} ${PLAN_GROUP} ${ROBOT}_ikfast_${PLAN_GROUP}_plugin ${IK_CPP}
popd

rosed ${ROBOT}_moveit_config kinematics.yaml

fi # (if-Usage)
