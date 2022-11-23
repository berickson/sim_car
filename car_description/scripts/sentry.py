#! /usr/bin/env python3
import math
import time # Time library
 
from geometry_msgs.msg import PoseStamped, Pose, Quaternion # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
from tf_transformations import quaternion_from_euler
 

def wait_to_complete(navigator):
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            #print(feedback)
            print('Distance remaining: ' + '{0:.2f}'.format(
                  feedback.distance_remaining))
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')

# begins navigatator moving to pose
def navigate_to_pose(navigator, pose):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose = pose
    navigator.goToPose(goal_pose)

def orientation_from_yaw(yaw):
    a = quaternion_from_euler(0, 0, yaw)
    q = Quaternion()
    q.x=a[0]
    q.y=a[1]
    q.z=a[2]
    q.w=a[3]
    return q

def pose_from_x_y_theta(x,y,theta) :
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.orientation = orientation_from_yaw(float(theta))
    return pose

def main():
    rclpy.init()
    node = rclpy.create_node('sentry')

    node.get_logger().info("started sentry")
    navigator = BasicNavigator()

    poses = []
    poses.append(pose_from_x_y_theta( 0.0,   0.0, 0.0))
    poses.append(pose_from_x_y_theta( 3.3,   0.5, 0.0))
    poses.append(pose_from_x_y_theta(-5.0,   0.0, math.pi))
    poses.append(pose_from_x_y_theta(-5.0,   5.0, math.pi/2))
    poses.append(pose_from_x_y_theta( 5.0,   5.0, math.pi))
    poses.append(pose_from_x_y_theta( 10.0, -5.0, math.pi))

    #navigator.waitUntilNav2Active(localizer="")
    while True:
        for pose in poses:
          navigate_to_pose(navigator, pose)
          wait_to_complete(navigator)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.w = 1.0

    navigator.goToPose(goal_pose)
    wait_to_complete(navigator)

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)






main()