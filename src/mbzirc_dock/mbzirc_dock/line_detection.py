from typing import List, Optional, Tuple

import matplotlib.pyplot as plt

import numpy as np
import math

import cv2 as cv

from skimage.transform import (hough_line, hough_line_peaks)

from mbzirc_dock.line import Line

import json


def Pi(ph: np.ndarray) -> np.ndarray:
    return ph[:-1, :] / ph[-1, :]


def Pi_inv(pih: np.ndarray) -> np.ndarray:
    n_points = pih.shape[1]
    return np.vstack((pih, np.ones((1, n_points))))


def est_line(x: np.ndarray) -> np.ndarray:
    """Estimates line from points using PCA.
    :param x - 2xn nonhomogenous points"""
    d = np.cov(x)[:, 0]
    d /= np.linalg.norm(d)
    l = [d[1], -d[0]]
    l.append(-(l @ x.mean(1)))
    return np.array(l)


def separate(line: np.ndarray, points: np.ndarray, threshold: float) -> Tuple[np.ndarray, np.ndarray]:
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


def all_points_valid(line: np.ndarray, points: np.ndarray, threshold: float) -> int:
    """Checks if all points are inliers.
    :param line - 3x1 homogenous line
    :param points - 3xn homogenous points
    :param threshold - maximum distance from line to be counted as inlier

    :return number of inliers"""
    inliers, outliers = separate(line, Pi_inv(points), threshold)
    return inliers.shape[1] == points.shape[1], inliers, outliers


def detect_lines(points: np.ndarray, k: int, dist_threshold: float, log) -> List[Line]:
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
            new_line = est_line(iter_points)

            if all_points_valid(new_line, iter_points, dist_threshold):
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

def filter_lines(points, prev_ang): 
        """ Line detection using hough lines """

        ## Parameters ##
        grid_resolution = 0.1       # Resolution of the grid (adjust based on your needs)
        dilate_kernel_size = 3      # Kernel size to apply dilate to laser scan image to detect lines
        angle_difference = 10       # Angle difference between previous best_line detected and the current one
                                    # to avoid selecting a different best line detected that is not the target
        points_in_line_th_1 = 0.2  # Threshold to select points that belong to the first line detected
        points_in_line_th_2 = 0.05  # Threshold to select points that belong to the final line

        best_line = None

        # Calculate the grid size based on the range of coordinates
        min_x, min_y = np.min(points, axis=1)
        max_x, max_y = np.max(points, axis=1)

        grid_size_x = int((max_x - min_x) / grid_resolution) + 1
        grid_size_y = int((max_y - min_y) / grid_resolution) + 1

        # Create an empty grid
        occupancy_grid = np.zeros((grid_size_x, grid_size_y), dtype=np.uint8)

        # Map laser scan points to the grid
        for x, y in points.T:  # Transpose 'points' to loop over columns
            x_index = int((x - min_x) / grid_resolution)
            y_index = int((y - min_y) / grid_resolution)
            occupancy_grid[x_index, y_index] = 255  # Set as occupied

        # Create image to be processed
        image = occupancy_grid

        # Dilate image to join continuous points that define a line
        kernel = np.ones((dilate_kernel_size,dilate_kernel_size),np.uint8)
        image = cv.dilate(image,kernel,iterations = 1)

        # Classic straight-line Hough transform
        h, theta, d = hough_line(image)

        # Generating figure 1
        """ fig, axes = plt.subplots(1, 3, figsize=(15, 6))
        ax = axes.ravel()

        ax[0].imshow(image, cmap=cm.gray)
        ax[0].set_title('Input image')
        ax[0].set_axis_off()

        ax[1].imshow(np.log(1 + h),
                    extent=[np.rad2deg(theta[-1]), np.rad2deg(theta[0]), d[-1], d[0]],
                    cmap=cm.gray, aspect=1/1.5)
        ax[1].set_title('Hough transform')
        ax[1].set_xlabel('Angles (degrees)')
        ax[1].set_ylabel('Distance (pixels)')
        ax[1].axis('image')

        ax[2].imshow(image, cmap=cm.gray) """

        min_line_dist = 9999999

        # Obtain first and last point of each line detected in the image
        lines_hough = []
        for _, angle, dist in zip(*hough_line_peaks(h, theta, d)):
            # First and last line point in image frame
            y0 = (dist - 0 * np.cos(angle)) / np.sin(angle)
            y1 = (dist - (image.shape[1] - 1) * np.cos(angle)) / np.sin(angle)
            x0 = 0
            x1 = image.shape[1] - 1
            #ax[2].plot((x0, x1), (y0, y1), '-r')
            
            # Convert line points detected to laser scan frame
            p1 = [y0*grid_resolution+min_x, x0*grid_resolution+min_y]
            p2 = [y1*grid_resolution+min_x, x1*grid_resolution+min_y]
            
            # Create line with first and last point
            line = [p1, p2]
            lines_hough.append([p1, p2])

            #line_msg = self._visualize_line(line[0], line[1], id=0)

            # Get all laser points that belongs to the line
            valid_points = points_on_line_with_threshold(points, line, points_in_line_th_1)

            # Convert points to an array
            valid_points_array = np.transpose(np.array(valid_points))
            
            if len(valid_points_array) == 0: 
                continue

            # Estimate line from scan points using PCA
            valid_points_eq = est_line(valid_points_array)

            # Get inliers from that line
            _, inliers, _ = all_points_valid(valid_points_eq, valid_points_array, threshold=points_in_line_th_2)
            # Update line cluster eliminating outliers
            valid_points_array = inliers[:2,:]
            valid_points_eq = est_line(valid_points_array)

            # Find start and end line point

            # Compute the orthogonal vector [-B, A]
            orthogonal_vector = np.array([-valid_points_eq[1], valid_points_eq[0]])

            # Project all points onto the orthogonal vector and sort them
            projections = np.dot(valid_points_array.T, orthogonal_vector)
            sorted_indices = np.argsort(projections)

            # The first point is the one with the smallest projection value, and the last point is the one with the largest projection value
            first_point = valid_points_array[:, sorted_indices[0]]
            last_point = valid_points_array[:, sorted_indices[-1]]  

            # Get x axis line angle
            x1, y1 = first_point
            x2, y2 = last_point
            slope = (y2-y1)/(x2-x1)
            angle_x_axis = np.rad2deg(math.atan(slope))

            #lower, higher = self._order_line_endpoints(first_point, last_point)
            h = np.linalg.norm(last_point)
            l = np.linalg.norm(first_point)
            r = np.linalg.norm(last_point - first_point)
            curr_angle = np.arccos((r ** 2 + l ** 2 - h ** 2) / (2 * r * l))

            line_i = Line(eq= valid_points_eq, start=first_point, end=last_point)

            if prev_ang == None: 
                prev_ang = angle_x_axis

            print(f"Prev angle: {prev_ang}")
            print(f"Curr angle: {angle_x_axis}")
            print(f"Ang diff:   {abs(angle_x_axis - prev_ang)}")

            # Select as best line if is the closest and the angle does not differ more than x degrees with the previous best_line
            if np.linalg.norm(line_i.center()) < min_line_dist and abs(angle_x_axis - prev_ang) < angle_difference: 
                min_line_dist = np.linalg.norm(line_i.center())
                best_line = line_i

                """ lines_array_msg = MarkerArray()
                lines_array_msg.markers = []
                for i in range(valid_points_array.shape[1]): 
                    x = valid_points_array[0, i]
                    y = valid_points_array[1, i]
                    marker = self.get_marker([x, y], color=[255.0, 0.0, 0.0], scale=0.1, id_=i)
                    lines_array_msg.markers.append(marker)
                self.scan_pub.publish(lines_array_msg) """

        print(f"Start: {best_line.start}")
        print(f"End: {best_line.end}")

        # Read the existing JSON data from the file, if any
        existing_points = []

        try:
            with open('points.json', 'r') as jsonfile:
                existing_points = json.load(jsonfile)
        except FileNotFoundError:
            # The file doesn't exist yet, so we'll start with an empty list.
            existing_points = []

        # Append the new points to the existing data
        existing_points.extend((best_line.end[0], best_line.end[1]))

        # Write the updated data (old and new points) back to the JSON file
        with open('points.json', 'w') as jsonfile:
            json.dump(existing_points, jsonfile)

    
        return best_line

def distance_point_to_line(point, line):
    x1, y1 = line[0]
    x2, y2 = line[1]
    x0, y0 = point

    numerator = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
    denominator = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    if denominator == 0:
        return float('inf')  # Avoid division by zero if the line has zero length
    
    return numerator / denominator

def points_on_line_with_threshold(point_set, line, threshold):
    result = []
    for i in range(point_set.shape[1]):
        point = [point_set[0,i], point_set[1,i]]
        distance = distance_point_to_line(point, line)
        if distance <= threshold:
            result.append(point)
    return result