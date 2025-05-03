import math

import cv2
import matplotlib as mpl
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
import os
from scipy.spatial import ConvexHull

initial_sigma = 1
initial_threshold1 = 50
initial_threshold2 = 150
kernel = np.ones((5, 5), np.uint8)
min_ratio = 0.25
max_dist = 30


def blur(gray_image):
    """Applies gaussian blur with emphasis on the center

    Args:
        gray_image: The image to be blurred

    Returns:
        The blurred image
    """
    center_x, center_y = gray_image.shape[1] // 2, gray_image.shape[0] // 2
    radius = 150  # Radius of the clear area (adjust as needed)

    # Apply Gaussian blur to the entire image
    blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 1)

    # Create a radial gradient mask
    Y, X = np.ogrid[:gray_image.shape[0], :gray_image.shape[1]]
    dist_from_center = np.sqrt((X - center_x) ** 2 + (Y - center_y) ** 2)
    mask = (dist_from_center <= radius).astype(np.float32)

    # Blend the original and blurred images
    return (gray_image * mask + blurred_image * (1 - mask)).astype(np.uint8)


def process_image(image):
    """ The main function that process the image

    Args:
        image: The original image

    Returns:
        Convex hull of the segmented object
    """
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (3, 3), initial_sigma)
    # opening = cv2.morphologyEx(blurred_image, cv2.MORPH_OPEN, kernel)
    edges = cv2.Canny(blurred_image, initial_threshold1, initial_threshold2)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hulls = []
    for contour in contours:
        hull = cv2.convexHull(contour)
        hulls.append(hull)
    modified_hulls = hull_seg(hulls, gray_image.shape)
    return modified_hulls


def merge_two_hulls(hull1, hull2):
    """ Connects two convex hull into one convex hull that containes both

    Args:
        hull1: First hull
        hull2: Second Hull

    Returns:
        Merged convex hull
    """
    combined_points = np.concatenate((hull1, hull2), axis=0)
    return cv2.convexHull(combined_points)


def min_distance(hull1, hull2):
    """ Returns the min distance between 2 convex hulls.

    Args:
        hull1: First hull
        hull2: Second Hull

    Returns:
        The distance between the hulls
    """
    min_dist = float('inf')
    for point1 in hull1:
        for point2 in hull2:
            dist = np.linalg.norm(point1 - point2)
            if dist < min_dist:
                min_dist = dist
    return min_dist


def check_ratio(hull: np.ndarray):
    """ Check if the ratio between the height and the width of the object is reasonable

    Args:
        hull (np.ndarray): The Convex hull we check

    Returns:
        Whether it's ratios are valid for our usecase. (To filter noise in segmentation)
    """
    rect = cv2.minAreaRect(hull)
    w, h = rect[1]
    if w == 0 or h == 0:
        return False
    aspect_ratio = w / h
    return min_ratio < aspect_ratio < (1 / min_ratio)


def hull_in_center(hull, img_shape):
    """ Check whether a convex hull is roughly in the middle of the image

    Args:
        hull: The convex hull
        img_shape: The dimensions of the image

    Returns:
        Whether the hull's center of mass is in between the 1st to 3rd quarters of the width.
    """
    height, width = img_shape
    left_boundary = width / 4
    right_boundary = 3 * width / 4
    center_mass, dim, angle = cv2.minAreaRect(hull)
    x, y = center_mass

    # Check if the hull bounds are within the middle third
    x_condition = left_boundary <= x <= right_boundary
    left_boundary = height / 4
    right_boundary = 3 * height / 4
    y_condition = left_boundary <= y <= right_boundary
    return x_condition


def hull_seg(hulls, img_shape):
    """ Segment Convex hull from preproccessed image to the object estimated hull.

    Args:
        hulls: The convex hulls after canny edge detection
        img_shape: The dimensions of the image

    Returns:
        The segmented hull inside an array.
    """
    merged = True
    while merged:
        merged = False
        largest_hull = None
        largest_area = -1
        for hull in hulls:
            area = cv2.contourArea(hull)
            if area > largest_area and check_ratio(hull):
                largest_area = area
                largest_hull = hull
        if largest_area == -1:
            break

        new_hulls = []
        tmp_hull = largest_hull
        for hull in hulls:
            if not np.array_equal(largest_hull, hull):
                ret, _ = cv2.intersectConvexConvex(tmp_hull, hull)
                d = min_distance(tmp_hull, hull)
                if ret > 0:
                    if check_ratio(hull) and hull_in_center(hull, img_shape):
                        tmp_hull = merge_two_hulls(tmp_hull, hull)
                elif d < max_dist:
                    if check_ratio(hull) and hull_in_center(hull, img_shape):
                        rect = cv2.minAreaRect(hull)
                        w, h = rect[1]
                        if w == 0 or h == 0:
                            continue
                        tmp_hull = merge_two_hulls(tmp_hull, hull)

                else:
                    new_hulls.append(hull)
        new_hulls.append(tmp_hull)
        hulls = new_hulls
    return [tmp_hull]


def project_point_hull(hull: ConvexHull, p):
    """ Project a 2d point onto it's closest point on a convex hull

    Args:
        hull (ConvexHull): The hull we project the point onto
        p : The 2d point

    Returns:
        The projected point on the hull
    """
    min_d = float('inf')
    closest_point = None
    normal = None
    for simplex in hull.simplices:
        p1, p2 = hull.points[simplex]

        edge_vector = p2 - p1
        edge_length = np.linalg.norm(edge_vector)

        edge_direction = edge_vector / edge_length

        p1_to_query = p - p1

        projection_length = np.dot(p1_to_query, edge_direction)

        if 0 <= projection_length <= edge_length:
            projection = p1 + projection_length * edge_direction
        else:
            projection = p1 if np.linalg.norm(p - p1) < np.linalg.norm(p - p2) else p2

        distance = np.linalg.norm(p - projection)

        if distance < min_d:
            closest_point = projection
            min_d = distance
            normal = np.array([-edge_direction[1], edge_direction[0]])
            normal = normal / np.linalg.norm(normal)

    return closest_point, min_d, normal


def cosine_similarity(A, B):
    """Calculated Cosine Similarity between 2 vectors.

    Args:
        A: The first vector
        B: The second vector

    Returns:
        The cosine similarity of A,B.
    """
    dot_product = np.dot(A, B)
    norm_A = np.linalg.norm(A)
    norm_B = np.linalg.norm(B)
    similarity = dot_product / (norm_A * norm_B)
    return similarity


def angle_heuristic(angle):
    """A regularization heuristic to incentivize neutral angles (optimal would be 0 degrees relative to x axis or 180 degrees)

    Args:
        angle: The angle on which we return the heuristic

    Returns:
        The heuristic value for the given angle.
    """
    return (abs(np.pi / 2 - (angle % np.pi)) - np.pi / 2) / np.pi


def in_hull(p, hull: ConvexHull):
    """ Returns whether a point is in the convex hull.

    Args:
        p: The point
        hull (ConvexHull): The convex hull

    Returns:
        True if p is inside hull
    """
    return hull.find_simplex(p) >= 0


def find_best_angle(hull, center, radius):
    """ Compares discrete set of angles and returns the one with the lowest angle heuristic+
     +maximal simalrity of the angle with the normal of the object's convex hull. It is also requiring the angle to make sure the arm wont intersect the object.

    Args:
        hull: The object's convex hull
        center (_type_): center of the arm
        radius (_type_): The rotation radius of the arm

    Returns:
        The best angle out of the set.
    """
    from scipy.spatial import Delaunay
    del_hull = Delaunay(hull.points)
    best_angle = 0
    best_similarity = float('-inf')
    for angle in range(-180, 15, 15):
        angle_radians = angle * np.pi / 180
        normal = np.array([np.cos(angle_radians), np.sin(angle_radians)])
        if in_hull(center + radius * normal, del_hull):
            continue

        p, d, hull_normal = project_point_hull(hull, normal)
        similarity = abs(cosine_similarity(normal, hull_normal))
        print(f'angle {angle} similarity {similarity} angle heuristic {angle_heuristic(angle_radians)}')
        if similarity + angle_heuristic(angle_radians) > best_similarity + angle_heuristic(best_angle):
            print(f'best angle {angle} similarity {similarity} angle heuristic {angle_heuristic(angle_radians)}')

            best_angle = angle_radians
            best_similarity = similarity

    return best_angle, best_similarity


def sample_hull(hull, sample_size=3):
    """ Sample points from a convex hull

    Args:
        hull: The hull we sample from
        sample_size: How many points we sample. Defaults to 3.

    Returns:
        The sampled points
    """
    return hull.squeeze()[np.random.choice(hull.shape[0], size=sample_size)]

print(cv2.GaussianBlur(np.array([[0, 0, 0],
                  [0, 100, 0],
                  [0, 0, 00]], dtype=np.uint8), (3, 3), 1))
kernel = cv2.getGaussianKernel(3, 1.0)

# Convert to 2D kernel by multiplying it by its transpose
kernel_2d = kernel @ kernel.T

# Print the 3x3 kernel
print(kernel_2d)
