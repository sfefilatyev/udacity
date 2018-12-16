"""Show 3D histogram."""

import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot3d(pixels, colors_rgb,  # pylint: disable=dangerous-default-value
           axis_labels=list("RGB"), axis_limits=[(0, 255), (0, 255), (0, 255)]):
    """Plot pixels in 3D."""

    # Create figure and 3D axis_handlees
    fig = plt.figure(figsize=(8, 8))
    axis_handle = Axes3D(fig)

    # Set axis_handleis limits
    axis_handle.set_xlim(*axis_limits[0])
    axis_handle.set_ylim(*axis_limits[1])
    axis_handle.set_zlim(*axis_limits[2])

    # Set axis_handleis labels and sizes
    axis_handle.tick_params(axis='both', which='major', labelsize=14, pad=8)
    axis_handle.set_xlabel(axis_labels[0], fontsize=16, labelpad=16)
    axis_handle.set_ylabel(axis_labels[1], fontsize=16, labelpad=16)
    axis_handle.set_zlabel(axis_labels[2], fontsize=16, labelpad=16)

    # Plot pixel values with colors given in colors_rgb
    axis_handle.scatter(
        pixels[:, :, 0].ravel(),
        pixels[:, :, 1].ravel(),
        pixels[:, :, 2].ravel(),
        c=colors_rgb.reshape((-1, 3)), edgecolors='none')

    return axis_handle  # return Axes3D object for further manipulation


# Read a color image
#img = cv2.imread("000275.png")
#img = cv2.imread("25.png")
img = cv2.imread("31.png")
#img = cv2.imread("53.png")
#img = cv2.imread("8.png")
# Select a small fraction of pixels to plot by subsampling it
scale = max(img.shape[0], img.shape[1], 64) / 64  # at most 64 rows and columns
img_small = cv2.resize(img, (np.int(img.shape[1] / scale), np.int(img.shape[0] / scale)),
                       interpolation=cv2.INTER_NEAREST)

# Convert subsampled image to desired color space(s)
img_small_RGB = cv2.cvtColor(img_small, cv2.COLOR_BGR2RGB)  # OpenCV uses BGR, matplotlib likes RGB
img_small_HSV = cv2.cvtColor(img_small, cv2.COLOR_BGR2HSV)
img_small_LUV = cv2.cvtColor(img_small, cv2.COLOR_BGR2LUV)
img_small_rgb = img_small_RGB / 255.  # scaled to [0, 1], only for plotting

# Plot and show
plot3d(img_small_RGB, img_small_rgb)
plt.show()

plot3d(img_small_HSV, img_small_rgb, axis_labels=list("HSV"))
plt.show()

plot3d(img_small_LUV, img_small_rgb, axis_labels=list("LUV"))
plt.show()
