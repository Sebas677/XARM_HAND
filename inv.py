#!/usr/bin/env python3


#libraries declaration
from __future__ import print_function  # Printing
import rospy  # Python client library
import actionlib  # ROS action library
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)  # Controller messages
from std_msgs.msg import Float64  # 64-bit floating point numbers
from trajectory_msgs.msg import JointTrajectoryPoint  # Robot trajectories
from geometry_msgs.msg import Point
import math


def callback(msg):
    global x, y, z
    x = msg.x
    y = msg.y
    z = msg.z
    rospy.loginfo("coordinates: x=%f y=%f z=%f" % (x, y, z))

#axis limitations to avoid malfunctioning
    if z <=0.5:
        z=0.5
    if y <=0.5:
        y=0.5

#array which contains values and desired value signs for orientation
    joint_values = [x, y, -z]

    # Limit joint values to desired range
    joint_limits = [
        (-2.0, 2.0),  # joint1 limits
        (-1.5, 1.5),  # joint2 limits
        (-1.5, 1.5),  # joint3 limits
        (-1.5, 1.5),  # joint4 limits
        (-1.5, 1.5),  # joint5 limits
    ]

#joints adjustment for precision and malfunctioning avoidance
    clamped_joint_values = []
    for i in range(len(joint_values)):
        clamped_value = max(joint_values[i], joint_limits[i][0])
        clamped_value = min(clamped_value, joint_limits[i][1])
        clamped_joint_values.append(clamped_value)

    # Create an action client to send joint trajectory goals
    arm_client = actionlib.SimpleActionClient(
        "/xarm/xarm5_traj_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction,
    )

    # Wait for the server to start up and start listening for goals.
    arm_client.wait_for_server()

    # Create a new goal to send to the Action Server
    arm_goal = FollowJointTrajectoryGoal()

    # Store the names of each joint of the robot arm
    arm_goal.trajectory.joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
    ]

    # Create a trajectory point
    point = JointTrajectoryPoint()

    # Store the desired joint values
    point.positions = clamped_joint_values
    #point.positions = clamped_joint_values

    # Set the time it should in seconds take to move the arm to the desired joint angles
    point.time_from_start = rospy.Duration(0.5)

    # Add the desired joint values to the goal
    arm_goal.trajectory.points.append(point)

    # Define timeout values
    exec_timeout = rospy.Duration(10)
    prmpt_timeout = rospy.Duration(5)

    # Send a goal to the ActionServer and wait for the server to finish performing the action
    arm_client.send_goal_and_wait(arm_goal, exec_timeout, prmpt_timeout)


if __name__ == "__main__":
    rospy.init_node("robot_mov", anonymous=True)
    rospy.Subscriber("/position", Point, callback)
    rospy.spin()
