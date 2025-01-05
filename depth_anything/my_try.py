import cv2
import torch

from depth_anything_v2.dpt import DepthAnythingV2

# model_configs = {
#     'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
#     'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
#     'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]}
# }
#
# encoder = 'vitb' # or 'vits', 'vitb'
# dataset = 'hypersim' # 'hypersim' for indoor model, 'vkitti' for outdoor model
# max_depth = 20 # 20 for indoor model, 80 for outdoor model
#
# model = DepthAnythingV2(**{**model_configs[encoder], 'max_depth': max_depth})
# model.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_metric_{dataset}_{encoder}.pth', map_location='cpu'))
# model.eval()
#
# raw_img = cv2.imread('./bottle.PNG')
# depth = model.infer_image(raw_img) # HxW depth map in meters in numpy
# print(depth.shape)
# print(depth.max())

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
In terminal for creating point cloud:
python depth_to_pointcloud.py --encoder vitb --load-from checkpoints/depth_anything_v2_metric_hypersim_vitb.pth --max-depth 20 --img-path './bottle.PNG' --outdir 'checkpoints'
"""
