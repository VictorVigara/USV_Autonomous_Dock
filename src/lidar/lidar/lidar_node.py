import rclpy
from rclpy.node import Node
from rclpy.time import Time


from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2 #used to read points 
import  numpy as np

from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


import math




class Line:
    def __init__(self, eq: np.ndarray, start: np.ndarray, end: np.ndarray) -> None:
        self.start = start
        self.end = end

    def length(self) -> float:
        return np.linalg.norm(self.end - self.start)

    def center(self) -> np.ndarray:
        return (self.start + self.end) / 2

class LidarScan(Node):
    def __init__(self):
        super().__init__("lidar_node")
        self.get_logger().info("LidarScan initialized!")
        self.BASE_LINK = 'usv/base_link'

        self.laser_sub = self.create_subscription(
            PointCloud2, '/cloud_fullframe', self._laser_cb, 10)
        self.center_pub = self.create_publisher(Marker, 'target_center', 10)
        self.scan_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.line_pub = self.create_publisher(Marker, 'line', 10)
        self.traj_pub = self.create_publisher(Marker, 'trajectory', 10)


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


    def _laser_cb(self, pointcloud: PointCloud2):
        """ PointCloud2 callback"""
        
        # Get the center_line from the PointCloud2
        center_line = self._pc2_to_scan(pointcloud=pointcloud, ang_threshold=[1, -1])

        points = np.array(center_line).T # 3xN points
        points = points[:2, :]

        # Detect continuous lines in the pointcloud        
        lines_detected = self.detect_lines(points = points[:2, :], k = 4, dist_threshold=0.05)


        # Get line length information to filter them
        marker_id = 0
        best_line = lines_detected[0]
        line_detected = False
        max_length = 0
        for i in range(len(lines_detected)): 
            marker_id += 1
            line_i = lines_detected[i]
            line_i_length = line_i.length()
            line_i_center = line_i.center()

            if line_i_length > max_length: 
                max_length_idx = i
            # Plot in Rviz the center point of the filtered lines
            print(f"Line center: {line_i_center}; Line length: {line_i_length}")
            if line_i_length > 0.7 and line_i_length < 1.0: 
                print(f"Line center: {line_i_center}; Line length: {line_i_length}")
                marker = self.get_marker(line_i_center, color=[0.0, 0.0, 255.0], scale=0.1, id_=marker_id)
                self.center_pub.publish(marker)
                best_line = lines_detected[i]
                line_detected = True

        if line_detected: 
            lower, higher = self._order_line_endpoints(best_line.start, best_line.end)
            self._visualize_line(lower, higher)
            h = np.linalg.norm(higher)
            l = np.linalg.norm(lower)
            r = np.linalg.norm(higher - lower)
            angle = np.arccos((r ** 2 + l ** 2 - h ** 2) / (2 * r * l))
            self.get_logger().info(f'Line angle: {np.rad2deg(angle):.1f}°; h: {h}; l: {l}; r:{r}')
        else: 
            # Calculate the angle for the biggest line
            # THIS IS NOT USEFUL JUST FOR DEBUGGING
            lower, higher = self._order_line_endpoints(lines_detected[max_length_idx].start, lines_detected[max_length_idx].end)
            self._visualize_line(lower, higher, color=[0.0, 0.5, 0.5])
            h = np.linalg.norm(higher)
            l = np.linalg.norm(lower)
            r = np.linalg.norm(higher - lower)
            angle = np.arccos((r ** 2 + l ** 2 - h ** 2) / (2 * r * l))
            self.get_logger().info(f'Line angle: {np.rad2deg(angle):.1f}°; h: {h}; l: {l}; r:{r}')

        

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
            marker = self.get_marker(scan_line[i], color=[0.0, 255.0, 0.0], scale=0.05, id_=id)
            marker_array.markers.append(marker)
        self.scan_pub.publish(marker_array)
        return np.array(scan_line)


    def _scan_to_points(self, scan: LaserScan) -> np.ndarray:
        """Converts a laser scan to a 2D array of points in the base_link frame."""
        n_points = len(scan.ranges)
        #T = self._get_transform(self.BASE_LINK, scan.header.frame_id)

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
        #points_base = T @ points_lidar
        points_base = points_lidar
        return points_base[:2, :]   # drop z
    
    def get_marker(self, point: np.ndarray, color = None, stamp=None, id_=0, scale=None) -> Marker:
        """Returns a marker for the given point."""
        if color is None:
            color = (1.0, 0.0, 0.0)

        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        if scale is None: 
            scale = 0.3

        msg = Marker()
        msg.header.stamp = stamp
        msg.header.frame_id = self.BASE_LINK
        msg.type = Marker.SPHERE
        msg.id = id_
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = msg.scale.z = scale
        msg.color.r = color[0]
        msg.color.g = color[1]
        msg.color.b = color[2]
        msg.color.a = 1.0

        return msg
    
    def Pi(self, ph: np.ndarray) -> np.ndarray:
        return ph[:-1, :] / ph[-1, :]


    def Pi_inv(self, pih: np.ndarray) -> np.ndarray:
        n_points = pih.shape[1]
        return np.vstack((pih, np.ones((1, n_points))))


    def est_line(self, x: np.ndarray) -> np.ndarray:
        """Estimates line from points using PCA.
        :param x - 2xn nonhomogenous points"""
        d = np.cov(x)[:, 0]
        d /= np.linalg.norm(d)
        l = [d[1], -d[0]]
        l.append(-(l @ x.mean(1)))
        return np.array(l)


    def separate(self, line: np.ndarray, points: np.ndarray, threshold: float) -> Tuple[np.ndarray, np.ndarray]:
        """Separates inliers and outliers on the line given threshold.
        :param line - 3x1 homogenous line
        :param points - 3xn homogenous points
        :param threshold - maximum distance from line to be counted as inlier

        :return separated inliers and outliers"""

        line_scaled = line / np.linalg.norm(line[:2])
        dists = np.abs(line_scaled.T @ points)
        inliers = points[:, dists <= threshold]
        outliers = points[:, dists > threshold]

        return inliers, outliers


    def all_points_valid(self, line: np.ndarray, points: np.ndarray, threshold: float) -> int:
        """Checks if all points are inliers.
        :param line - 3x1 homogenous line
        :param points - 3xn homogenous points
        :param threshold - maximum distance from line to be counted as inlier

        :return number of inliers"""
        inliers, _ = self.separate(line, self.Pi_inv(points), threshold)
        return inliers.shape[1] == points.shape[1]

    def detect_lines(self, points: np.array, k: int, dist_threshold: float) -> List[Line]:
        """Detects continuous lines in ordered points set.
        :param points - 2xn ORDERED homogenous points
        :param k - number of consecutive points to estimate line
        :param dist_threshold - maximum distance from line to be counted as inlier

        :return List of Line obejcts"""
        n = points.shape[1]
        lines = []

        current_line: Optional[np.ndarray] = None

        i = 0
        while i < n - k + 1:
            j = k
            while i + j < n:
                iter_points = points[:, i:i + j]
                new_line = self.est_line(iter_points)  # Removed 'self.' from here

                if self.all_points_valid(new_line, iter_points, dist_threshold):  # Removed 'self.' from here
                    current_line = new_line
                    j += 1
                else:
                    if current_line is not None:
                        start_ = points[:, i]
                        end_ = points[:, i + j - 2]
                        line = Line(current_line, start_, end_)
                        lines.append(line)
                    i = i + j - 1
                    current_line = None
                    break

            # we exited loop without breaking, i.e. reached end of range, need to break
            if current_line is not None:
                start_ = points[:, i]
                end_ = points[:, i + j - 1]
                line = Line(current_line, start_, end_)
                lines.append(line)
                break
            if i + j >= n:
                break

        return lines
    
    def _visualize_line(self, lower, higher, color = [1.0, 0.5, 0.5]) -> None:
        """Visualizes the tracked line."""
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.BASE_LINK
        msg.scale.x = 0.1
        msg.type = Marker.LINE_LIST
        msg.points = []
        msg.color.r = color[0]
        msg.color.g = color[1]
        msg.color.b = color[2]
        msg.color.a = 1.0

        start_ = Point(x=lower[0], y=lower[1], z=0.0)
        end_ = Point(x=higher[0], y=higher[1], z=0.0)
        msg.points.append(start_)
        msg.points.append(end_)

        self.line_pub.publish(msg)

    @staticmethod
    def _order_line_endpoints(first: np.ndarray, second: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Orders the endpoints of a line so that the second is to the left of the line
        defined by the origin and the first point."""
        if np.cross(first, second) > 0:
            return first, second
        else:
            return second, first

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


def main(args=None):
    rclpy.init(args=args)
    node = LidarScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()