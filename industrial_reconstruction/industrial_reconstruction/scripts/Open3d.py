import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d


def read_mesh(path):
    return o3d.io.read_triangle_mesh(path)


def read_pcd(path):
    return o3d.io.read_point_cloud(path)


if __name__ == "__main__":
    mesh = read_pcd("/home/justin/ply_files/path2.ply")

    o3d.visualization.draw_geometries([mesh])

    downpcd = mesh.voxel_down_sample(voxel_size=0.05)
