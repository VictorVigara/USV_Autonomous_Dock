import enum
import time
import traceback
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2 #used to read points 

import math


from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from mbzirc_interfaces.action import Dock

from mbzirc_dock.approach_policy import ApproachPolicy
from mbzirc_dock.line_detection import detect_lines
from mbzirc_dock.orbit_policy import OrbitPolicy
from mbzirc_dock.policy import Policy, PolicyAction, PolicyState, PolicyCanceledException, PolicyOutOfBoundsException
from mbzirc_dock.stop_policy import StopPolicy
from mbzirc_dock.target_policy import TargetPolicy
from mbzirc_dock.touch_policy import TouchPolicy
from mbzirc_dock.tracker import Tracker2D


class DockStage(enum.Enum):
    STOP = enum.auto()
    ENTER_ORBIT = enum.auto()
    ORBIT = enum.auto()
    APPROACH = enum.auto()
    TOUCH = enum.auto()


class DockActionServer(Node):
    BASE_LINK = 'usv/base_link'
    DT = 0.1

    def __init__(self):
        super().__init__('dock_action_server')

        # parameters for the orbit
        self.r_orbit = 10.0
        self.v_orbit = 2.0

        # parameters for the line detection
        self.line_thres_min = 0.7
        self.line_thres_max = 1
        self.angle_threshold = np.deg2rad(50) # 150

        # parameters for line tracking
        self.P0 = 5.0
        self.R0 = 0.05
        self.Q0 = 0.01
        self.mode = 'v'
        self.tracked_start: Optional[Tracker2D] = None
        self.tracked_end: Optional[Tracker2D] = None

        # parameters for the approach policy
        self.trajectory_relative = [
            np.array([[-8.0,  6.0]]).T,
            np.array([[-6.0,  5.0]]).T,
            np.array([[-4.0,  4.0]]).T,
            np.array([[-2.0,  3.5]]).T,
            np.array([[ 0.0,  3.5]]).T,
            np.array([[ 2.0,  3.5]]).T,
        ]
        self.trajectory_targets = [
            np.array([[0.0, 0.0]]).T for _ in range(len(self.trajectory_relative))
        ]
        self.trajectory_speeds = [
            2.0,
            2.0,
            2.0,
            2.0,
            1.0,
            0.5
        ]

        # parameters for the touch policy
        self.usv_width = 3.4
        self.v_touch = 0.5
        self.touch_precision = 0.3

        # variables initialization
        self.state = DockStage.STOP
        self.target_center = np.zeros(2)
        self.points = np.zeros((2, 1))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._default_policy = StopPolicy()
        self._policy_queue: deque[Policy] = deque()
        self._current_state = PolicyState(
            time=self.get_clock().now(),
            position=np.zeros(2),
            velocity=np.zeros(2),
            acceleration=np.zeros(2),
            target=np.zeros(2)
        )

        """ self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self._laser_cb, 10) """
        self.laser_sub = self.create_subscription(
            PointCloud2, '/cloud_fullframe', self._laser_cb, 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.center_pub = self.create_publisher(Marker, 'target_center', 10)
        self.target_pub = self.create_publisher(Marker, 'target', 10)
        self.traj_pub = self.create_publisher(Marker, 'trajectory', 10)
        self.line_pub = self.create_publisher(Marker, 'line', 10)
        self.scan_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.control_timer = self.create_timer(self.DT, self._control_cb)

        self._action_server = ActionServer(
            self, Dock, 'dock',
            self._execute_callback)

    def _control_cb(self):
        """Control loop, called every 0.1 seconds. Executes current policy action."""
        try:
            self._current_state.target = self.target_center

            self._track_line()
            
            print(self._policy_queue)
            action = self._get_action(self._current_state)

            msg = Twist()
            msg.linear.x = action.linear_velocity[0]
            msg.linear.y = action.linear_velocity[1]
            msg.linear.z = action.linear_velocity[2]
            msg.angular.x = action.angular_velocity[0]
            msg.angular.y = action.angular_velocity[1]
            msg.angular.z = action.angular_velocity[2]
            self.twist_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Uncaught exception: {traceback.format_exc()}')

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> Dock.Result:
        """Callback for the action server."""
        try:
            self.get_logger().info('Executing goal...')

            policy = TargetPolicy(self.v_orbit, self.r_orbit, 2.0, self.get_logger())
            self._policy_queue.append(policy)

            orbit_policy = OrbitPolicy(self.r_orbit, self.v_orbit, self.get_logger())
            self._policy_queue.append(orbit_policy)

            self.state = DockStage.APPROACH
            while not self.state == DockStage.STOP:
                time.sleep(self.DT)

            goal_handle.succeed()
            result = Dock.Result(success=True)
            return result
        except Exception as e:
            self.get_logger().error(f'Uncaught exception: {traceback.format_exc()}')
            goal_handle.abort()
            return Dock.Result(success=False)

    def _laser_cb(self, pointcloud: PointCloud2) -> None:
        """Callback for the laser scan topic."""
        # Get the center_line from the PointCloud2
        center_line = self._pc2_to_scan(pointcloud=pointcloud, ang_threshold=[1, -1])
        points = np.array(center_line).T # 3xN points
        self.points = points[:2, :]

        #self.points = self._scan_to_points(scan)
        self.target_center = np.median(self.points, axis=1)
        marker = self.get_marker(self.target_center)

        self.center_pub.publish(marker)

    def _pc2_to_scan(self, pointcloud: PointCloud2, ang_threshold = [-0.5, 5]): 
        """Get an horizontal scan from a PointCloud2"""
        angles = []
        scan_line = []
        for p in point_cloud2.read_points(pointcloud, field_names = ("x", "y", "z"), skip_nans=True):
            # Get XYZ coordinates to calculate vertical angle and filter by vertical scans
            x = p[0]
            y = p[1]
            z = p[2]
            d = math.sqrt(x**2 + y**2 + z**2)
            if d != 0: 
                # Calculate vertical angle
                angle = 90 - np.rad2deg(np.arccos(z/d))

                # Save points from center scan
                if angle < ang_threshold[0] and angle > ang_threshold[1] and x > 1 and x < 4 and d < 2.5: 
                    scan_line.append([x, y, z])
                    

                # Calculate vertical angles from the PointCloud2
                """ ang_saved = False
                if len(angles) == 0: 
                    angles.append(angle)
                    print(angles)
                else: 
                    for i in range(len(angles)): 
                        diff = angle - angles[i]
                        
                        if diff < 0.5 and diff > -0.5:
                            ang_saved = True
                    
                    if ang_saved == False: 
                        angles.append(angle) """
        id = 0
        marker_array = MarkerArray()
        for i in range(len(scan_line)): 
            id += 1
            marker = self.get_marker(scan_line[i], color=[0.0, 255.0, 0.0], id_=id, scale = 0.05)
            marker_array.markers.append(marker)
        self.scan_pub.publish(marker_array)
        return np.array(scan_line)


    def _track_line(self) -> None:
        """Detects and tracks a line in the laser scan."""
        """ if self.state == DockStage.STOP:
            return """

        lines = detect_lines(self.points, 4, 0.1, self.get_logger())

        if len(lines) == 0:
            return

        best_line = lines[0]
        for line in lines:
            if line.length() > best_line.length():
                best_line = line

        if best_line.length() < self.line_thres_min and best_line.length() > self.line_thres_max:
            self.get_logger().debug(f'Line not found')
            return

        if self.tracked_start is not None:
            self._visualize_line()

        lower, higher = self._order_line_endpoints(best_line.start, best_line.end)
        h = np.linalg.norm(higher)
        l = np.linalg.norm(lower)
        r = np.linalg.norm(higher - lower)
        angle = np.arccos((r ** 2 + l ** 2 - h ** 2) / (2 * r * l))
        self.get_logger().info(f'Line angle: {np.rad2deg(angle):.1f}°')

        if angle < self.angle_threshold and self.tracked_start is None:
            self.get_logger().warn(f'Too small line angle: {np.rad2deg(angle):.1f}° < {np.rad2deg(self.angle_threshold):.1f}°, ignoring...')
            return

        if self.tracked_start is not None:
            self.tracked_start.update(lower[:, None])
            self.tracked_start.predict()
            self.tracked_end.update(higher[:, None])
            self.tracked_end.predict()
        else:
            self.get_logger().info(f'Started tracking line!')
            self.state = DockStage.APPROACH

            self.tracked_start = Tracker2D(
                lower, self.P0, self.R0, self.Q0, self.DT, self.mode)
            self.tracked_end = Tracker2D(
                higher, self.P0, self.R0, self.Q0, self.DT, self.mode)

            self._policy_queue.clear()
            for target, speed in zip(self.trajectory_targets, self.trajectory_speeds):
                policy = ApproachPolicy(
                    target, speed, 1.0, self.get_logger()
                )
                self._policy_queue.append(policy)

            touch_policy = TouchPolicy(
                self.tracked_start, self.tracked_end, self.usv_width,
                self.v_touch, self.touch_precision,
                self._set_stop,
                self.get_logger()
            )
            self._policy_queue.append(touch_policy)
            self.get_logger().info(f'Initial x0: {self.tracked_start.x}')
        self._recalculate_approach_trajectory(lower, higher)

    def _set_stop(self) -> None:
        """Callback for the touch policy."""
        self.state = DockStage.STOP
        self.tracked_start = None
        self.tracked_end = None
        self._policy_queue.clear()

    def _recalculate_approach_trajectory(self, lower: np.ndarray, higher: np.ndarray) -> None:
        """Recalculates the approach trajectory based on the detected line."""
        xa = (higher - lower) / np.linalg.norm(higher - lower)
        R = np.eye(2)
        R[0:2, 0] = xa
        R[0, 1] = -R[1, 0]
        R[1, 1] = R[0, 0]

        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.BASE_LINK
        msg.scale.x = msg.scale.y = msg.scale.z = 0.3
        msg.type = Marker.SPHERE_LIST
        msg.points = []
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.b = 0.5
        msg.color.a = 1.0

        for i in range(len(self.trajectory_relative)):
            p = self.trajectory_relative[i]
            self.trajectory_targets[i][:] = lower[:, None] + R @ p

            msg.points.append(
                Point(x=self.trajectory_targets[i][0, 0], y=self.trajectory_targets[i][1, 0], z=0.0)
            )

        self.traj_pub.publish(msg)

    @staticmethod
    def _order_line_endpoints(first: np.ndarray, second: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Orders the endpoints of a line so that the second is to the left of the line
        defined by the origin and the first point."""
        if np.cross(first, second) > 0:
            return first, second
        else:
            return second, first

    def _get_action(self, state: PolicyState) -> PolicyAction:
        """Returns the action for the first policy in the queue."""
        success = False

        while not success:
            # use default policy if nothing else is in the queue
            if len(self._policy_queue) == 0:
                self._policy_queue.append(self._default_policy)
            elif len(self._policy_queue) >= 2 and self._policy_queue[0] is self._default_policy:
                self._policy_queue.popleft()

            policy = self._policy_queue[0]
            print(policy)
            try:
                action = policy.get_action(state)
                if isinstance(policy, TargetPolicy):
                    end = policy.end
                    self.get_logger().info(f'end point: {end}')
                    marker = self.get_marker(end)
                    self.target_pub.publish(marker)
                success = True
                break
            except (PolicyCanceledException, PolicyOutOfBoundsException) as e:
                self.get_logger().info(f'Error {e}\nRemoving from queue')
                policy.cancel()
                if len(self._policy_queue) > 0:
                    self._policy_queue.popleft()

        return action

    def _scan_to_points(self, scan: LaserScan) -> np.ndarray:
        """Converts a laser scan to a 2D array of points in the base_link frame."""
        n_points = len(scan.ranges)
        T = self._get_transform(self.BASE_LINK, scan.header.frame_id)

        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, n_points)
        angles = angles[np.isfinite(ranges)]
        ranges = ranges[np.isfinite(ranges)]
        n_good_points = len(ranges)

        points_lidar = np.array([
            ranges * np.cos(angles),
            ranges * np.sin(angles),
            np.zeros(n_good_points),
            np.ones(n_good_points)
        ])
        points_base = T @ points_lidar
        return points_base[:2, :]   # drop z

    def get_marker(self, point: np.ndarray, color = None, stamp=None, id_=0, scale = 0.3) -> Marker:
        """Returns a marker for the given point."""
        if color is None:
            color = (1.0, 0.0, 0.0)

        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        msg = Marker()
        msg.header.stamp = stamp
        msg.header.frame_id = self.BASE_LINK
        msg.type = Marker.SPHERE
        msg.id = id_
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = msg.scale.z = scale
        msg.color.r = color[0]
        msg.color.g = color[1]
        msg.color.b = color[2]
        msg.color.a = 1.0

        return msg

    def _visualize_line(self) -> None:
        """Visualizes the tracked line."""
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.BASE_LINK
        msg.scale.x = 0.1
        msg.type = Marker.LINE_LIST
        msg.points = []
        msg.color.r = 1.0
        msg.color.g = 0.5
        msg.color.b = 0.5
        msg.color.a = 1.0

        start_ = Point(x=self.tracked_start.pos[0], y=self.tracked_start.pos[1], z=0.0)
        end_ = Point(x=self.tracked_end.pos[0], y=self.tracked_end.pos[1], z=0.0)
        msg.points.append(start_)
        msg.points.append(end_)

        self.line_pub.publish(msg)

    def _get_transform(self, parent: str, child: str, time: rclpy.time.Time = Time()) -> np.ndarray:
        """Returns the transformation matrix from parent to child frame."""
        transform = self.tf_buffer.lookup_transform(
            parent,
            child,
            time=time
        )
        rot = Rotation.from_quat([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ]).as_matrix()
        trans = np.array([
            [transform.transform.translation.x],
            [transform.transform.translation.y],
            [transform.transform.translation.z]
        ])
        T = np.hstack((rot, trans))
        T = np.vstack((T, [0, 0, 0, 1]))
        return T
    
