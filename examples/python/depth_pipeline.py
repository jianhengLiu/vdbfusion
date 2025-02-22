#!/usr/bin/env python3
# @file      kitti_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
from tqdm import trange
import argh

from datasets import DepthPoseDataset as Dataset
from vdbfusion_pipeline import VDBFusionPipeline as Pipeline
import open3d as o3d
import numpy as np


def main(
    ouster_scans: str,
    sequence: int = 0,
    config: str = "./examples/python/config/depth_pose.yaml",
    n_scans: int = -1,
    jump: int = 0,
    visualize: bool = False,
):
    """Help here!"""
    dataset = Dataset(ouster_scans, config)
    pipeline = Pipeline(dataset, config, jump=jump, n_scans=n_scans, map_name="depth_pose")
    pipeline.run()
    pipeline.visualize() if visualize else None

    # # integrate all scans and show it
    # # Create Open3D point cloud for integration
    # integrated_pcd = o3d.geometry.PointCloud()
    # for idx in trange(len(dataset), unit=" frames"):
    #     scan, pose = dataset[idx]
    #     # Convert scan to Open3D point cloud
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(scan)
    #     # Transform point cloud using pose
    #     # pcd.transform(pose)
    #     # Combine with integrated cloud
    #     integrated_pcd += pcd

    # # Visualize the integrated point cloud
    # o3d.visualization.draw_geometries([integrated_pcd])


if __name__ == "__main__":
    argh.dispatch_command(main)
