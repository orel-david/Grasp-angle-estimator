import cv2
import matplotlib as mpl
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
import os
from scipy.spatial import ConvexHull

from segmentation import utils
from segmentation.utils import hull_seg, process_image

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


# for img in images:
#     img_cv = cv2.imread(img)
#     img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB)
#     contour_image(img_cv)

def draw_circle(image, center, radius, color=(255, 255, 255), thickness=2):
    """
    Draws a circle on the given image.

    Args:
        image (np.ndarray): The image on which to draw the circle.
        center (tuple): (x, y) coordinates of the circle center.
        radius (int): Radius of the circle.
        color (tuple): Color of the circle in (B, G, R). Default is white.
        thickness (int): Thickness of the circle border. Use -1 for a filled circle.

    Returns:
        np.ndarray: Image with the circle drawn.
    """
    return cv2.circle(image, center, radius, color, thickness)


# Create a blank image
mug_img = cv2.imread('set/MUG2.jpg')

image = np.zeros_like(mug_img)
mug_object = process_image(mug_img)
print(mug_img.shape)
# Define the center and radius
center = mug_img.shape[1] // 2 +0, mug_img.shape[0] // 2
print(center)
radius = 100

# Draw the circle on the image
image_with_circle = draw_circle(image, center, radius, color=(0, 255, 0), thickness=3)
grip_angle, similarity = 0, 0
for hull in mug_object:
    cv2.drawContours(image_with_circle, [hull], -1, 255, thickness=1)
    grip_angle, similarity = utils.find_best_angle(ConvexHull(hull.squeeze()), center, radius)
# Convert from BGR to RGB for matplotlib
image_with_circle_rgb = cv2.cvtColor(image_with_circle, cv2.COLOR_BGR2RGB)

cv2.line(image_with_circle_rgb, center,
         np.round(center + radius * np.array([np.cos(grip_angle), np.sin(grip_angle)])).astype(int),
         (255, 0, 0), 5)
# Display using matplotlib
plt.imshow(image_with_circle_rgb)
plt.axis("off")  # Hide axes
plt.show()
