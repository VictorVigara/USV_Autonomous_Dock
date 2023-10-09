import rclpy
from rclpy.node import Node
from rclpy.time import Time


from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
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
        self.BASE_LINK = 'world'

        self.laser_sub = self.create_subscription(
            PointCloud2, '/cloud_fullframe', self._laser_cb, 1)
        self.center_pub = self.create_publisher(Marker, 'target_center', 10)
        self.scan_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.line_pub = self.create_publisher(Marker, 'line', 10)
        self.traj_pub = self.create_publisher(Marker, 'trajectory', 10)
        self.pc_crop_pub = self.create_publisher(PointCloud2, 'pc_crop', 10)

        # parameters crop pc2
        self.x_crop = 2.2
        self.y_crop = 1.3

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
        
        
        min_dist = 999999
        # Detect lines that are closer (assuming that the target line is the closest one)
        for line in lines_detected: 
            center = line.center()
            dist = math.sqrt(center[0]**2 + center[1]**2)
            if dist < min_dist: 
                best_line = line

        lower, higher = self._order_line_endpoints(best_line.start, best_line.end)
        #line_msg = self._visualize_line(lower, higher, id=id)
        h = np.linalg.norm(higher)
        l = np.linalg.norm(lower)
        r = np.linalg.norm(higher - lower)
        angle = np.rad2deg(np.arccos((r ** 2 + l ** 2 - h ** 2) / (2 * r * l)))
        print(f"Target line angle: {angle}")
        
        # Get all the points from the pointcloud that belongs to the detected line and are within a threshold
        line_cluster = self.points_on_line_with_threshold(points, [lower, higher], threshold = 1)
        line_cluster_array = np.transpose(np.array(line_cluster))
        # Calculate the line adjusted to those points through PCA
        line_cluster_eq = self.est_line(line_cluster_array)

        # Get inliers from that line
        _, inliers, _ = self.all_points_valid(line_cluster_eq, line_cluster_array, threshold=0.1)
        # Update line cluster eliminating outliers
        line_cluster_array = inliers[:2,:]
        line_cluster_eq = self.est_line(line_cluster_array)
        best_line = Line(line_cluster_eq, line_cluster_array[:,0], line_cluster_array[:,-1])

        points = line_cluster_array.T

        # Compute the orthogonal vector [-B, A]
        orthogonal_vector = np.array([-line_cluster_eq[1], line_cluster_eq[0]])

        # Project all points onto the orthogonal vector and sort them
        projections = np.dot(line_cluster_array.T, orthogonal_vector)
        sorted_indices = np.argsort(projections)

        # The first point is the one with the smallest projection value, and the last point is the one with the largest projection value
        first_point = line_cluster_array[:, sorted_indices[0]]
        last_point = line_cluster_array[:, sorted_indices[-1]]


        _ = self._visualize_line(first_point, last_point, id = 0, color=[0.0, 1.0, 0.0])
        lines_array_msg = MarkerArray()
        lines_array_msg.markers = []
        for i in range(line_cluster_array.shape[1]): 
            x = line_cluster_array[0, i]
            y = line_cluster_array[1, i]
            marker = self.get_marker([x, y], color=[255.0, 0.0, 0.0], scale=0.1, id_=i)
            lines_array_msg.markers.append(marker)
        self.scan_pub.publish(lines_array_msg)

        a = 1

        # Get line length information to filter them
        """ marker_id = 0
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
            self.get_logger().info(f'Line angle: {np.rad2deg(angle):.1f}°; h: {h}; l: {l}; r:{r}') """

    def distance_point_to_line(self, point, line):
        x1, y1 = line[0]
        x2, y2 = line[1]
        x0, y0 = point

        numerator = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
        denominator = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        
        if denominator == 0:
            return float('inf')  # Avoid division by zero if the line has zero length
        
        return numerator / denominator

    def points_on_line_with_threshold(self, point_set, line, threshold):
        result = []
        for i in range(point_set.shape[1]):
            point = [point_set[0,i], point_set[1,i]]
            distance = self.distance_point_to_line(point, line)
            if distance <= threshold:
                result.append(point)
        return result

    def _pc2_to_scan(self, pointcloud: PointCloud2, ang_threshold = [-0.5, 5]): 
        """Get an horizontal scan from a PointCloud2"""
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
                if (angle < ang_threshold[0] and angle > ang_threshold[1]) and ((x > self.x_crop) or (x < -self.x_crop) or (x > -self.x_crop and x < self.x_crop and (y > self.y_crop or y < -self.y_crop))): 
                    scan_line.append([x, y, z])
                    
        pc2_cropped = PointCloud2()
        pc2_cropped.header = pointcloud.header
        pc2_cropped.height = 1
        pc2_cropped.width = len(scan_line)
        pc2_cropped.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        pc2_cropped.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        pc2_cropped.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        pc2_cropped.is_bigendian = False
        pc2_cropped.point_step = 12  # 4 (x) + 4 (y) + 4 (z) bytes per point
        pc2_cropped.row_step = pc2_cropped.point_step * len(scan_line)
        pc2_cropped.data = np.array(scan_line, dtype=np.float32).tobytes()
        self.pc_crop_pub.publish(pc2_cropped)
        print("publishing pc")
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
        inliers, outliers = self.separate(line, self.Pi_inv(points), threshold)
        return inliers.shape[1] == points.shape[1], inliers, outliers

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
        k = 2
        
        while i < n - k + 1:
            j = k
            n_last_outliers = 0 # number of outliers at the end of the line being analyzed
            map_outliers = {}   # {key: j, value: n_outliers}
            check_points = points[:, i:i+j]
            differences_x = np.diff(np.array(check_points[0, :]))
            differences_y = np.diff(np.array(check_points[0, :]))
            # check if the first 2 points are consecutive
            
            while i + j < n:
                # Check if the first 2 points are close enough
                if j == 2 and (abs(differences_x) > 0.3 and abs(differences_y) > 0.3):
                    i += 1
                    break

                previous_keys = []
                iter_points = points[:, i:i + j]
                # Compute the differences between consecutive points
                differences_x = np.diff(np.array(iter_points[0,:]))
                differences_y = np.diff(np.array(iter_points[1,:]))
                # Calculate the median of the differences
                median_distance_x = np.median(np.abs(differences_x))
                median_distance_y = np.median(np.abs(differences_y))
                # Calculate last points distance in x and y
                last_dist_x = abs(iter_points[0,-2]-iter_points[0,-1])
                last_dist_y = abs(iter_points[1,-2]-iter_points[1,-1])


                # If map outliers dict is not empty check if the last value is 0
                if len(map_outliers.keys()) is not 0 and len(map_outliers.keys()) is not 1: 
                    # Get last j_index and check if n_last_oytliers is 0
                    last_j = list(map_outliers.keys())[-1]
                    if map_outliers[last_j] == 0:
                        # Find the previous keys whose values are not 0 to delete outliers to calculate the line
                        previous_keys = [key for key in map_outliers.keys() if map_outliers[key] != 0]
                    non_last_outliers = sum(1 for value in map_outliers.values() if value == 0)
                    if map_outliers[last_j] != 0 and non_last_outliers/len(map_outliers)>0.7: 
                        # Delete last outliers from line estimation
                        previous_keys = [key for key in map_outliers.keys() if map_outliers[key] != 0]
                
                if len(previous_keys) is not 0:
                    # Delete outlier points from iter_points
                    mask = np.ones(iter_points.shape[1], dtype=bool)
                    for j_value in previous_keys:
                        if j_value <= iter_points.shape[1]:
                            # Set the indices corresponding to j_value to False in the mask
                            mask[j_value - 1] = False

                    # Apply the mask to iter_points to get a new iter_points vector
                    new_iter_points = iter_points[:, mask]
                else: 
                    new_iter_points = iter_points

                # Estimate the line of the points being analyzed 
                new_line = self.est_line(new_iter_points)  # Removed 'self.' from here
                # Check if points being analyzed belong to the estimated line
                valid_flag, inliers, outliers = self.all_points_valid(new_line, iter_points, dist_threshold)
                # Visualize the line being analyzed
                #line_msg = self._visualize_line(lower=iter_points[:,0], higher=iter_points[:,-1], color=[1.0, 0.0, 0.0], id=99)
                


                if abs(last_dist_x) > 4 and last_dist_y > 4 and valid_flag == False:
                    # Get the values associated with the target_key
                    j_values = list(map_outliers.values())
                    # Check if any of the values are zero
                    if any(j_value == 0 for j_value in j_values) and len(j_values) > 1:
                        # Use last j value that had n_last_outliers = 0
                        max_j_with_zero_last_outliers = int(max((key for key, value in map_outliers.items() if value == 0), default=None))
                        j = max_j_with_zero_last_outliers
                    if current_line is not None:
                        start_ = points[:, i]
                        end_ = points[:, i + j - 1]
                        line = Line(current_line, start_, end_)
                        lines.append(line)
                    i = i + j
                    current_line = None
                    break

                # Check if the last point is an outlier
                for p in range(outliers.shape[1]): 
                    if iter_points[0,-1] == outliers[0, p]: 
                        n_last_outliers += 1
                # If there are no outliers or the last point is not an outlier update line
                if valid_flag or n_last_outliers == 0:  # Removed 'self.' from here
                    current_line = new_line
                    map_outliers[j] = n_last_outliers
                    j += 1

                # If the last point is an outlier, check if there are more consecutive outliers
                # If there are more than x outliers at the end of the line finish line
                elif n_last_outliers < 20:
                    n_last_outliers = 0
                    for l in range(iter_points.shape[1]):
                        if n_last_outliers == l: 
                            for p in range(outliers.shape[1]): 
                                if iter_points[0,-(l+1)] == outliers[0, p]: 
                                    n_last_outliers += 1
                                    
                        else: 
                            break
                    # Save j indexes and n_last_outliers to reject outliers in case the line finish
                    map_outliers[j] = n_last_outliers
                    j+=1   
                else: 

                    # Get the values associated with the target_key
                    j_values = list(map_outliers.values())

                    # Check if any of the values are zero
                    if any(j_value == 0 for j_value in j_values):
                        # Use last j value that had n_last_outliers = 0 to create the correct line
                        max_j_with_zero_last_outliers = int(max((key for key, value in map_outliers.items() if value == 0), default=None))
                        j = max_j_with_zero_last_outliers

                    if current_line is not None:
                        start_ = points[:, i]
                        end_ = points[:, i + j - 1]
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
            if i + k >= n:
                break
        return lines
    
    def _visualize_line(self, lower, higher, id, color = [1.0, 0.5, 0.5]) -> None:
        """Visualizes the tracked line."""
        msg = Marker()
        msg.id = id
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
        return msg

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