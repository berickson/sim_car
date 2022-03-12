#! /usr/bin/env python3
import time # Time library
 
from geometry_msgs.msg import PoseStamped, Pose # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult # Helper module
 

def wait_to_complete(navigator):
    i = 0
    while not navigator.isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # # Some navigation timeout to demo cancellation
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #     navigator.cancelTask()

            # # Some navigation request change to demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose.pose.position.x = -3.0
            #     navigator.goToPose(goal_pose)

# begins navigatator moving to pose
def navigate_to_pose(navigator, pose):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose = pose
    navigator.goToPose(goal_pose)


def main():
    rclpy.init()
    node = rclpy.create_node('sentry')

    node.get_logger().error("abc")

    node.get_logger().info("started sentry")
    print("1")
    navigator = BasicNavigator()
    print("2")
    #help(navigator)
    #navigator.waitUntilNav2Active(localizer='slam')
    print("3")

    pose_1 = Pose()
    pose_1.position.x = 0.0
    pose_1.position.y = 0.0
    pose_1.orientation.w = 1.0

    pose_2 = Pose()
    pose_2.position.x = -5.0
    pose_2.position.y = 0.0
    pose_2.orientation.w = 1.0

    while True:
        navigate_to_pose(navigator, pose_1)
        wait_to_complete(navigator)
        navigate_to_pose(navigator, pose_2)
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