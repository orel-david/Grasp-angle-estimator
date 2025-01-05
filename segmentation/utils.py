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
kernel = np.ones((3, 3), np.uint8)
min_ratio = 0.3
max_dist = 10


def blur(gray_image):
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
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (3, 3), initial_sigma)
    opening = cv2.morphologyEx(blurred_image, cv2.MORPH_OPEN, kernel)
    edges = cv2.Canny(opening, initial_threshold1, initial_threshold2)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hulls = []
    for contour in contours:
        hull = cv2.convexHull(contour)
        hulls.append(hull)
    modified_hulls = hull_seg(hulls)
    return modified_hulls


def compute_aspect_ratio(hull):
    x, y, w, h = cv2.boundingRect(hull)
    aspect_ratio = h / w
    return aspect_ratio


def merge_two_hulls(hull1, hull2):
    combined_points = np.concatenate((hull1, hull2), axis=0)
    return cv2.convexHull(combined_points)


def min_distance(hull1, hull2):
    min_dist = float('inf')
    for point1 in hull1:
        for point2 in hull2:
            dist = np.linalg.norm(point1 - point2)
            if dist < min_dist:
                min_dist = dist
    return min_dist


def check_ratio(hull):
    ratio = compute_aspect_ratio(hull)
    return min_ratio < ratio < (1 / min_ratio)


def hull_seg(hulls):
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
                if ret > 0 or d < max_dist:
                    if check_ratio(hull):
                        tmp_hull = merge_two_hulls(tmp_hull, hull)
                else:
                    new_hulls.append(hull)
        new_hulls.append(tmp_hull)
        hulls = new_hulls
    return [tmp_hull]


def project_point_hull(hull: ConvexHull, p):
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
    dot_product = np.dot(A, B)
    norm_A = np.linalg.norm(A)
    norm_B = np.linalg.norm(B)
    similarity = dot_product / (norm_A * norm_B)
    return similarity


def angle_heuristic(angle):
    return (abs(np.pi / 2 - (angle % np.pi)) - np.pi / 2) / np.pi


def in_hull(p, hull: ConvexHull):
    from scipy.spatial import Delaunay
    hull = Delaunay(hull.points)

    return hull.find_simplex(p) >= 0


def find_best_angle(hull, center, radius):
    best_angle = 0
    best_similarity = float('-inf')
    for angle in range(-180, 15, 15):
        angle_radians = angle * np.pi / 180
        normal = np.array([np.cos(angle_radians), np.sin(angle_radians)])
        if in_hull(center + radius * normal, hull):
            continue

        p, d, hull_normal = project_point_hull(hull, normal)
        similarity = abs(cosine_similarity(normal, hull_normal))
        print(f'angle {angle} similarity {similarity} angle heuristic {angle_heuristic(angle_radians)}')
        if similarity + angle_heuristic(angle_radians) > best_similarity + angle_heuristic(best_angle):
            print(f'best angle {angle} similarity {similarity} angle heuristic {angle_heuristic(angle_radians)}')

            best_angle = angle_radians
            best_similarity = similarity

    return best_angle, best_similarity
