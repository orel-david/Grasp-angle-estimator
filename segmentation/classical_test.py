import cv2
import matplotlib as mpl
import time
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
mpl.use('TkAgg')

# Load the image
image = cv2.imread('kitchen.jpg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Initial parameters
initial_sigma = 1
initial_threshold1 = 50
initial_threshold2 = 150


# Function to update the plot
def update(val):
    sigma = sigma_slider.val
    threshold1 = threshold1_slider.val
    threshold2 = threshold2_slider.val
    start = time.time()
    # Apply GaussianBlur with updated sigma
    blurred_image = cv2.GaussianBlur(image, (5, 5), sigma)
    edges = cv2.Canny(blurred_image, int(threshold1), int(threshold2))
    end = time.time()
    print(f'elapsed time {end-start}')
    # Update the images in the plot
    ax_edges.imshow(edges, cmap='gray')
    fig.canvas.draw_idle()


# Create the figure and axes
fig, (ax_original, ax_edges) = plt.subplots(1, 2, figsize=(10, 5))
blurred_image = cv2.GaussianBlur(image, (5, 5), initial_sigma)

# Display the original image
ax_original.set_title('Original Image')
ax_original.imshow(image, cmap='gray')
ax_original.axis('off')

# Display the initial Canny edges
blurred_image = cv2.GaussianBlur(image, (5, 5), initial_sigma)

edges = cv2.Canny(blurred_image, initial_threshold1, initial_threshold2)
ax_edges.set_title('Canny Edges')
ax_edges.imshow(edges, cmap='gray')
ax_edges.axis('off')

# Add sliders for sigma, threshold1, and threshold2
ax_sigma = plt.axes((0.25, 0.1, 0.65, 0.03), facecolor='lightgoldenrodyellow')
ax_threshold1 = plt.axes((0.25, 0.05, 0.65, 0.03), facecolor='lightgoldenrodyellow')
ax_threshold2 = plt.axes((0.25, 0.0, 0.65, 0.03), facecolor='lightgoldenrodyellow')

sigma_slider = Slider(ax_sigma, 'Sigma', 0.1, 5.0, valinit=initial_sigma, valstep=0.1)
threshold1_slider = Slider(ax_threshold1, 'Threshold1', 0, 255, valinit=initial_threshold1, valstep=1)
threshold2_slider = Slider(ax_threshold2, 'Threshold2', 0, 255, valinit=initial_threshold2, valstep=1)

# Connect the sliders to the update function
sigma_slider.on_changed(update)
threshold1_slider.on_changed(update)
threshold2_slider.on_changed(update)

# Display the interactive plot
plt.tight_layout()
plt.show()
