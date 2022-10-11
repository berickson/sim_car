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


def main():
    rclpy.init()
    node = rclpy.create_node('sentry')

    node.get_logger().info("started sentry")
    navigator = BasicNavigator()

    pose_1 = Pose()
    pose_1.position.x = 0.0
    pose_1.position.y = 0.0
    #help(Quaternion)
    print(quaternion_from_euler(0.0, 0.0, 0.))
    pose_1.orientation = orientation_from_yaw(0)

    pose_2 = Pose()
    pose_2.position.x = -5.0
    pose_2.position.y = 0.0
    pose_2.orientation = orientation_from_yaw(math.pi)

    pose_3 = Pose()
    pose_3.position.x = -5.0
    pose_3.position.y = 5.0
    pose_3.orientation = orientation_from_yaw(3*math.pi/2)

    pose_4 = Pose()
    pose_4.position.x = 5.0
    pose_4.position.y = 5.0
    pose_4.orientation = orientation_from_yaw(math.pi)

    pose_5 = Pose()
    pose_5.position.x = 10.0
    pose_5.position.y = -5.0
    pose_5.orientation = orientation_from_yaw(math.pi)

    

    poses = [pose_1, pose_2, pose_3, pose_4, pose_5]

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