import cv2
import matplotlib as mpl
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

# Load the image
image = cv2.imread('kitchen.jpg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Initial parameters
initial_sigma = 1
initial_threshold1 = 50
initial_threshold2 = 150
kernel = np.ones((3, 3), np.uint8)

# Create the figure and axes
fig, (ax_original, ax_edges) = plt.subplots(1, 2, figsize=(10, 5))
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

# Display the interactive plot
plt.tight_layout()
plt.show()
