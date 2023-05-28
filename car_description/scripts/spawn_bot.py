import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]

    req = SpawnEntity.Request()
    req.name = "car"
    req.xml = content
    req.robot_namespace = ""
    req.reference_frame = "world"

    while not cli.wait_for_service(timeout_sec=20.0):
        node.get_logger().info('service not available, waiting again...')

    while(node.get_clock().now().seconds() == 0):
        node.get_logger().info('waiting for clock')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
