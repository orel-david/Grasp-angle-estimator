from skimage import io, transform
import numpy as np
import matplotlib.pyplot as plt
from skimage import feature
from skimage.segmentation import active_contour


img = io.imread("../seq_images/seq_mug/20241223150053.jpg", as_gray=True)
img = transform.resize(img, (240, 240))

s = np.linspace(0, 2 * np.pi, 100)
r = 120 + 110 * np.sin(s)
c = 120 + 110 * np.cos(s)
init = np.array([r, c]).T

snake = active_contour(
    # gaussian(img, sigma=0.3, preserve_range=False),
    feature.canny(img),
    init,
    alpha=0.05,
    beta=4,
    gamma=0.005,
    max_px_move=1.1,
    convergence=0.05
)

fig, ax = plt.subplots(figsize=(7, 7))
ax.imshow(feature.canny(img), cmap=plt.cm.gray)
ax.plot(init[:, 1], init[:, 0], '--r', lw=3)
ax.plot(snake[:, 1], snake[:, 0], '-b', lw=3)
ax.set_xticks([]), ax.set_yticks([])
ax.axis([0, img.shape[1], img.shape[0], 0])

plt.show()

