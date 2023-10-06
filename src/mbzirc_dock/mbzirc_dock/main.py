import rclpy
from rclpy.executors import MultiThreadedExecutor

from mbzirc_dock.dock_action_server import DockActionServer


def main(args=None):
    rclpy.init(args=args)

    dock_action_server = DockActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(dock_action_server, executor=executor)

    dock_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
