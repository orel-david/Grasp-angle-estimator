import cv2
import matplotlib as mpl
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
import os

images = os.listdir('set')
images = [os.path.join('set', img) for img in images]
# Initial parameters
initial_sigma = 1
initial_threshold1 = 50
initial_threshold2 = 150
kernel = np.ones((3, 3), np.uint8)
min_ratio = 0.3
max_dist = 10


def contour_image(image):
    # Create the figure and axes
    fig, (ax_original, ax_edges, ax_hull, ax_seg) = plt.subplots(1, 4, figsize=(10, 5))
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (3, 3), initial_sigma)
    opening = cv2.morphologyEx(blurred_image, cv2.MORPH_OPEN, kernel)

    # Display the original image
    ax_original.set_title('Original Image')
    ax_original.imshow(image, cmap='gray')
    ax_original.axis('off')

    edges = cv2.Canny(opening, initial_threshold1, initial_threshold2)

    ax_edges.set_title('Canny Edges')
    ax_edges.imshow(edges, cmap='gray')
    ax_edges.axis('off')

    convex_hull_image = np.zeros_like(edges)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hulls = []
    for contour in contours:
        hull = cv2.convexHull(contour)
        hulls.append(hull)
        cv2.drawContours(convex_hull_image, [hull], -1, 255, thickness=1)

    ax_hull.set_title('Hull')
    ax_hull.imshow(convex_hull_image, cmap='gray')
    ax_hull.axis('off')

    segmentation_display = np.zeros_like(edges)
    modified_hulls = hull_seg(hulls)
    for hull in modified_hulls:
        cv2.drawContours(segmentation_display, [hull], -1, 255, thickness=1)

    ax_seg.set_title('segment')
    ax_seg.imshow(segmentation_display, cmap='gray')
    ax_seg.axis('off')
    ##Display the interactive plot
    plt.tight_layout()
    plt.show()


def compute_aspect_ratio(hull):
    x, y, w, h = cv2.boundingRect(hull)
    aspect_ratio = h / w
    return aspect_ratio


def merge_two_hulls(hull1, hull2):
    combined_points = np.concatenate((hull1, hull2), axis=0)
    return cv2.convexHull(combined_points)


def min_distance(hull1, hull2):
    # TODO: OPTIMIZE
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
    largest_hull = None

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
    return [largest_hull]


for img in images:
    img_cv = cv2.imread(img)
    img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB)
    contour_image(img_cv)
