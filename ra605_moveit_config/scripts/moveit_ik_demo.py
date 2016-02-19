#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # sim = False; # for RPi2
        sim = True;

        rospy.init_node('moveit_demo')
                
        # Initialize the move group for the right arm
        arm1 = moveit_commander.MoveGroupCommander('arm1')
                
        # Get the name of the end-effector link
        # end_effector_link = arm1.get_end_effector_link()
        arm1.set_end_effector_link('link6')
        end_effector_link = arm1.get_end_effector_link()
                        
        # Set the reference frame for pose targets
        reference_frame = 'base_link'
        # reference_frame = 'link5'
        
        # Set the right arm reference frame accordingly
        arm1.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        arm1.allow_replanning(False)
        
        # Allow some leeway in position (meters) and orientation (radians)
        arm1.set_goal_position_tolerance(0.01)
        arm1.set_goal_orientation_tolerance(0.05)
        
#        # Start the arm in the "init" pose stored in the SRDF file
#        arm1.set_named_target('init')
#        arm1.go()
#        if sim:
#            rospy.sleep(2)
               
#         # Set the target pose.  This particular pose has the gripper oriented horizontally
#         # 0.85 meters above the ground, 0.10 meters to the right and 0.20 meters ahead of 
#         # the center of the robot base.
#         target_pose = PoseStamped()
#         target_pose.header.frame_id = reference_frame
#         target_pose.header.stamp = rospy.Time.now()     
#         target_pose.pose.position.x = 0.20
#         target_pose.pose.position.y = 0.3
#         target_pose.pose.position.z = 0.55
#         target_pose.pose.orientation.x = 0.0
#         target_pose.pose.orientation.y = 0.0
#         target_pose.pose.orientation.z = 0.0
#         target_pose.pose.orientation.w = 1.0
        
        # Set the start state to the current state
        arm1.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
#        arm1.set_pose_target(target_pose, end_effector_link)

        # Shift the end-effector to the right 5cm
        arm1.shift_pose_target(2, -0.05, end_effector_link)  # roll
        
        # Plan the trajectory to the goal
        traj = arm1.plan()
        
        # Execute the planned trajectory
        arm1.execute(traj)
    
        if sim:
            rospy.sleep(2)
        
        # Shift (3/Roll) the end-effector to the left 5cm
        arm1.shift_pose_target(2, 0.05, end_effector_link)
        
        # Plan the trajectory to the goal
        traj = arm1.plan()
        
        # Execute the planned trajectory
        arm1.execute(traj)
    
        if sim:
            rospy.sleep(2)
        
        # Shift the end-effector to the right 5cm
        arm1.shift_pose_target(1, -0.05, end_effector_link)
        arm1.go()
        if sim:
            rospy.sleep(2)
  
#         # Rotate the end-effector 90 degrees
#         arm1.shift_pose_target(3, -1.57, end_effector_link)
#         arm1.go()
#         rospy.sleep(1)
#           
#         # Store this pose as the new target_pose
#         saved_target_pose = arm1.get_current_pose(end_effector_link)
#           
#         # Move to the named pose "vertical"
#         arm1.set_named_target('vertical')
#         arm1.go()
#         rospy.sleep(1)
#           
#         # Go back to the stored target
#         arm1.set_pose_target(saved_target_pose, end_effector_link)
#         arm1.go()
#         rospy.sleep(1)
           
        # Finish up in the init position  
#        arm1.set_named_target('init')
#        arm1.go()

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

    
    
