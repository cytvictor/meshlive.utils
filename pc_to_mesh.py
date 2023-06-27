import open3d as o3d
import numpy as np 
import os
from time import time

# load point cloud
spath = "../datasets/fsvvd/Chatting/Raw"
pcd = o3d.io.read_point_cloud("../datasets/fsvvd/Chatting/Raw/chatting_0_raw.ply")
print(pcd)

def reconstruct_mesh(pcd, method="alpha", alpha=0.02):
    """
    Reconstruct mesh from point cloud
    """
    if method == "alpha":
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    elif method == "normal":
        pcd.orient_normals_consistent_tangent_plane(100)
        radii = [0.005, 0.01, 0.02, 0.04]
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(radii))
    else:
        raise ValueError("Invalid method")
    return mesh

mesh = reconstruct_mesh(pcd, method="alpha", alpha=0.02)
# Write mesh result to file
o3d.io.write_triangle_mesh("../datasets/fsvvd/Chatting/Raw-Mesh/chatting_0_raw-alpha.ply", mesh, True)


mesh = reconstruct_mesh(pcd, method="normal")
o3d.io.write_triangle_mesh("../datasets/fsvvd/Chatting/Raw-Mesh/chatting_0_raw-normal.ply", mesh, True)
