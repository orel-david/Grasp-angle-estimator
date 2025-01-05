"""
Born out of Depth Anything V1 Issue 36
Make sure you have the necessary libraries installed.
Code by @1ssb

This script processes a set of images to generate depth maps and corresponding point clouds.
The resulting point clouds are saved in the specified output directory.

Usage:
    python script.py --encoder vitl --load-from path_to_model --max-depth 20 --img-path path_to_images --outdir output_directory --focal-length-x 470.4 --focal-length-y 470.4

Arguments:
    --encoder: Model encoder to use. Choices are ['vits', 'vitb', 'vitl', 'vitg'].
    --load-from: Path to the pre-trained model weights.
    --max-depth: Maximum depth value for the depth map.
    --img-path: Path to the input image or directory containing images.
    --outdir: Directory to save the output point clouds.
    --focal-length-x: Focal length along the x-axis.
    --focal-length-y: Focal length along the y-axis.
"""

import argparse
import cv2
import glob
import numpy as np
import open3d as o3d
import os
from PIL import Image
import torch

from depth_anything_v2.dpt import DepthAnythingV2


def cross_product(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Because the np.cross() by itself give warning.
    Compute the cross product of two numpy arrays.
    """
    return np.cross(a, b)


def mirror_point(a, b, c, d, points):
    """
    Code from: https://www.geeksforgeeks.org/mirror-of-a-point-through-a-3-d-plane/
    """
    x1, y1, z1 = points[:, 0], points[:, 1], points[:, 2]
    print(x1.shape, y1.shape, z1.shape)
    k = (-a * x1 - b * y1 - c * z1 - d) / float((a * a + b * b + c * c))
    x2 = a * k + x1
    y2 = b * k + y1
    z2 = c * k + z1
    x3 = 2 * x2 - x1
    y3 = 2 * y2 - y1
    z3 = 2 * z2 - z1

    return np.stack((x3, y3, z3), axis=-1).reshape(-1, 3)


def calc_plane(points, selected_points):
    #   TODO:: define the correct points using segmentation.
    #   TODO:: There is a problem with point that shares the same value.
    #   TODO:: make sure that there is no zero normal.
    """

    :param points: generated image 3D points.
    :param selected_points: points selected from segmentation to create the plane.
    :return: plane points and mirrored points of the object around the plane.
    """
    point1, point2, point3 = selected_points
    print(point1, point2, point3)

    # Create the plane using three points from our 3D space.
    vec1 = point2 - point1
    vec2 = point3 - point1

    normal_vec = cross_product(vec1, vec2)
    normal_vec /= np.linalg.norm(normal_vec)

    d = -np.dot(normal_vec, point1)

    # Added Plane points.
    x_range = np.linspace(-1, 1, 100)
    y_range = np.linspace(-1, 1, 100)
    xx, yy = np.meshgrid(x_range, y_range)
    zz = (-normal_vec[0] * xx - normal_vec[1] * yy - d) / normal_vec[2]
    plane_points = np.stack((xx, yy, zz), axis=-1).reshape(-1, 3)

    # Calc mirror points.
    # zz_dist = points[:, 2] - (-normal_vec[0] * points[:, 0] - normal_vec[1] * points[:, 1] - d) / normal_vec[2]
    # zz_mirrored = points[:, 2] - 2*zz_dist
    # mirrored_points = np.stack((points[:, 0], points[:, 1], zz_mirrored), axis=-1).reshape(-1, 3)
    mirrored_points = mirror_point(normal_vec[0], normal_vec[1], normal_vec[2], d, points)

    return plane_points, mirrored_points


# Courtesy of Chat-gpt - needed to be replaced with segmentation.
def select_points(image):
    selected_points = []

    def on_mouse_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse click
            selected_points.append((x, y))
            print(f"Point selected: ({x}, {y})")

    cv2.imshow("Select Points", image)
    cv2.setMouseCallback("Select Points", on_mouse_click)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return selected_points


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Generate depth maps and point clouds from images.')
    parser.add_argument('--encoder', default='vitl', type=str, choices=['vits', 'vitb', 'vitl', 'vitg'],
                        help='Model encoder to use.')
    parser.add_argument('--load-from', default='', type=str, required=True,
                        help='Path to the pre-trained model weights.')
    parser.add_argument('--max-depth', default=20, type=float,
                        help='Maximum depth value for the depth map.')
    parser.add_argument('--img-path', type=str, required=True,
                        help='Path to the input image or directory containing images.')
    parser.add_argument('--outdir', type=str, default='./vis_pointcloud',
                        help='Directory to save the output point clouds.')
    parser.add_argument('--focal-length-x', default=470.4, type=float,
                        help='Focal length along the x-axis.')
    parser.add_argument('--focal-length-y', default=470.4, type=float,
                        help='Focal length along the y-axis.')

    args = parser.parse_args()

    # Determine the device to use (CUDA, MPS, or CPU)
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

    # Model configuration based on the chosen encoder
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }

    # Initialize the DepthAnythingV2 model with the specified configuration
    depth_anything = DepthAnythingV2(**{**model_configs[args.encoder], 'max_depth': args.max_depth})
    depth_anything.load_state_dict(torch.load(args.load_from, map_location='cpu'))
    depth_anything = depth_anything.to(DEVICE).eval()

    # Get the list of image files to process
    if os.path.isfile(args.img_path):
        if args.img_path.endswith('txt'):
            with open(args.img_path, 'r') as f:
                filenames = f.read().splitlines()
        else:
            filenames = [args.img_path]
    else:
        filenames = glob.glob(os.path.join(args.img_path, '**/*'), recursive=True)

    # Create the output directory if it doesn't exist
    os.makedirs(args.outdir, exist_ok=True)

    # Process each image file
    for k, filename in enumerate(filenames):
        print(f'Processing {k + 1}/{len(filenames)}: {filename}')

        # Load the image
        color_image = Image.open(filename).convert('RGB')
        width, height = color_image.size

        # Read the image using OpenCV
        image = cv2.imread(filename)
        pred = depth_anything.infer_image(image, height)

        # Resize depth prediction to match the original image size
        resized_pred = Image.fromarray(pred).resize((width, height), Image.NEAREST)

        # Generate mesh grid and calculate point cloud coordinates
        x, y = np.meshgrid(np.arange(width), np.arange(height))
        x = (x - width / 2) / args.focal_length_x
        y = (y - height / 2) / args.focal_length_y
        z = np.array(resized_pred)
        points = np.stack((np.multiply(x, z), np.multiply(y, z), z), axis=-1).reshape(-1, 3)

        # Added.
        # Select 2D points manually
        selected_2d_points = select_points(image)

        # Find the points selected in the points array.
        selected_3d_points = []
        for px, py in selected_2d_points:
            idx = py * width + px
            selected_3d_points.append(points[idx])

        px, py = selected_2d_points[0]
        start_idx = py * width + px
        points = points[start_idx:, :]

        # Calculate the plane from the selected 3D points
        plane_points, mirror_points = calc_plane(points, selected_3d_points)

        total_points = np.vstack((points, plane_points, mirror_points))

        colors = np.array(color_image).reshape(-1, 3) / 255.0
        colors = colors[start_idx:, :]
        plane_colors = np.full_like(plane_points, [1, 0, 0])
        mirror_colors = colors
        colors = np.vstack((colors, plane_colors, mirror_colors))

        # Back to original code.
        # Create the point cloud and save it to the output directory
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(total_points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud(os.path.join(args.outdir, os.path.splitext(os.path.basename(filename))[0] + ".ply"),
                                 pcd)


if __name__ == '__main__':
    main()
