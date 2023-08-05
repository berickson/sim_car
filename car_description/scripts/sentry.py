#! /usr/bin/env python3
import math

from geometry_msgs.msg import PoseStamped, Pose, Quaternion
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import random


def wait_to_complete(navigator):
    reuse_line = "\r"
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f"{reuse_line}Distance remaining: {feedback.distance_remaining:5.2f}", end="")
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('\nGoal succeeded!')
    elif result == TaskResult.CANCELED:
        print('\nGoal was canceled!')
    elif result == TaskResult.FAILED:
        print('\nGoal failed!')


def yaw_from_pose(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)

    return euler_from_quaternion(quaternion)[2]


# begins navigatator moving to pose
def navigate_to_pose(navigator, pose, wait=True):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose = pose

    print("Navigating to pose "
          f"{pose.position.x:g},{pose.position.y:g} {yaw_from_pose(pose)*180/math.pi:g}°")
    navigator.goToPose(goal_pose)
    if wait:
        wait_to_complete(navigator)


def orientation_from_yaw(yaw):
    a = quaternion_from_euler(0, 0, yaw)
    q = Quaternion()
    q.x = a[0]
    q.y = a[1]
    q.z = a[2]
    q.w = a[3]
    return q


def pose_from_x_y_theta(x, y, theta):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.orientation = orientation_from_yaw(float(theta))
    return pose


def pose_from_x_y_qz_qw(x, y, qz, qw):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


def main():
    rclpy.init()
    node = rclpy.create_node('sentry')

    node.get_logger().info("started sentry")
    navigator = BasicNavigator()

    # nav these in order to create a starter map
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(1.96, 0.52, 0.014, 0.99))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(2.92, 0.71, -0.09, 0.99))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(5.09, 0.40, -0.54 , 0.83))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(4.68, -3.38, -0.98, 0.17))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(0.07, -4.75, -0.99, 0.09))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(-3.89, -4.21, -0.99, 0.09))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(-10.38, -1.29, 0.69, 0.71))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(-8.11, 6.75, 0.38, 0.92))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(-1.37, 8.28, -0.05, 0.99))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(9.27, 7.14, -0.65, 0.75))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(9.9, -3.44, -0.69, 0.722))
    navigate_to_pose(navigator, pose_from_x_y_qz_qw(1.06, -6.84, 0.99, 0.08))

    # nav these randomly to make it interesting
    poses = []
    poses.append(pose_from_x_y_theta(0.0, 0.0, 0.0))
    poses.append(pose_from_x_y_theta(3.3, 0.5, 0.0))
    poses.append(pose_from_x_y_theta(-5.0, 0.0, math.pi))
    poses.append(pose_from_x_y_theta(-5.0, 5.0, math.pi/2))
    poses.append(pose_from_x_y_theta(5.0, 5.0, 0.0))
    poses.append(pose_from_x_y_theta(10.0, -5.0, -math.pi))
    poses.append(pose_from_x_y_theta(-10.0, -5.0, math.pi))
    poses.append(pose_from_x_y_theta(-10.0, 5.0, 3.0*math.pi/2))
    poses.append(pose_from_x_y_theta(10.0, 5.0, 0))

    while True:
        pose = random.choice(poses)
        navigate_to_pose(navigator, pose)

main()
