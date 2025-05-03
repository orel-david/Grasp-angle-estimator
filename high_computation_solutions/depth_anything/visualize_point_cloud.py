import cv2
import torch

from depth_anything_v2.dpt import DepthAnythingV2
import open3d as o3d

# Read .ply file
input_file = "./checkpoints/bottle7.ply"
# Read the point cloud
pcd = o3d.io.read_point_cloud(input_file)

# Visualize the point cloud within open3d
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5,
                                                         origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcd, axis])

"""
Place an image in the folder containing this file, and change the name in the following command to the image path name.
bottle.PNG should be changed to the wanted image path.

In terminal for creating point cloud:
python depth_to_pointcloud.py --encoder vitb --load-from checkpoints/depth_anything_v2_metric_hypersim_vitb.pth --max-depth 20 --img-path './bottle.PNG' --outdir 'checkpoints'

For the mirrored point cloud:
python depth_to_mirrored_pointcloud.py --encoder vitb --load-from checkpoints/depth_anything_v2_metric_hypersim_vitb.pth --max-depth 20 --img-path './bottle.PNG' --outdir 'checkpoints'
"""



