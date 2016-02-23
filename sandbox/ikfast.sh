#!/bin/bash

if [ $# -eq 0 ]
then
    echo "Usage: ikfast.sh <ROBOT-NAME>"
else    

# Reference:
# http://docs.ros.org/indigo/api/moveit_ikfast/html/doc/ikfast_tutorial.html

ROBOT=$1
roscp ${ROBOT}_description ${ROBOT}.urdf ./
rosrun collada_urdf urdf_to_collada ${ROBOT}.urdf ${ROBOT}.dae
rosrun moveit_ikfast round_collada_numbers.py ${ROBOT}.dae ${ROBOT}.rounded.dae 5
openrave-robot.py ${ROBOT}.rounded.dae --info links

# usage:
# python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --help

python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py \
        --robot=${ROBOT}.rounded.dae --iktype=transform6d --baselink=0 --eelink=6 \
        --savefile=ikfast61_${ROBOT}.cpp

IK_CPP=${PWD}/ikfast61_${ROBOT}.cpp

pushd ~/catkin_ws/src
# @planning_group_name - arm1
PLAN_GROUP=arm1
# @moveit_ik_plugin_pkg - ${ROBOT}_ikfast_${PLAN_GROUP}_plugin
IK_PKG=${ROBOT}_ikfast_${PLAN_GROUP}_plugin
test -d ${ROBOT}/${IK_PKG} && /bin/rm -Rf ${ROBOT}/${IK_PKG}

if ! [ -d ${IK_PKG} ]
then 
    catkin_create_pkg ${IK_PKG}
    cd ~/catkin_ws
    catkin_make # add $IK_PKG to search path
fi
echo "rosrun moveit_ikfast create_ikfast_moveit_plugin.py ${ROBOT} ${PLAN_GROUP} ${ROBOT}_ikfast_${PLAN_GROUP}_plugin ${IK_CPP}"
rosrun moveit_ikfast create_ikfast_moveit_plugin.py \
    ${ROBOT} ${PLAN_GROUP} ${ROBOT}_ikfast_${PLAN_GROUP}_plugin ${IK_CPP}

# TODO: move IK_PKG to ROBOT(ra605/) directory
# TODO: mv ${IK_PKG} ${ROBOT}/
popd

rosed ${ROBOT}_moveit_config kinematics.yaml

fi # if-usage
