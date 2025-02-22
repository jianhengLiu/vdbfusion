import os
import numpy as np
import open3d as o3d

def load_depth_poses(poses_txt):
    """Load 4x4 poses from text file."""
    poses = []
    with open(poses_txt, "r") as f:
        lines = f.readlines()
    # Each pose is stored in 4 lines
    for i in range(0, len(lines), 4):
        matrix = []
        for j in range(4):
            row = [float(x) for x in lines[i + j].split()]
            matrix.append(row)
        poses.append(np.array(matrix))
    return poses

def get_ply_files(folder):
    """Get all .ply files from a folder, sorted by name."""
    return sorted(
        [f for f in os.listdir(folder) if f.endswith(".ply")],
        key=lambda x: int(x.split('.')[0])
    )

class DepthPoseDataset:
    def __init__(self, data_source, config):
        """
        data_source: path containing the 'depth' folder and 'depth_poses.txt'
        config: an object or dictionary with dataset settings (e.g. min_range, max_range)
        """
        self.config = config
        self.data_source = data_source
        self.scan_folder = os.path.join(self.data_source, "depths")
        self.pose_file = os.path.join(self.data_source, "depth_poses.txt")

        self.scan_files = get_ply_files(self.scan_folder)
        self.poses = load_depth_poses(self.pose_file)

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        # Returns a PointCloud(np.array(N, 3))
        # and sensor origin(np.array(3,))
        # in the global coordinate frame.
        
        ply_path = os.path.join(self.scan_folder, self.scan_files[idx])
        scan = o3d.io.read_point_cloud(ply_path)
        pose = self.poses[idx]

        # Get points in local frame
        points = np.asarray(scan.points)
        # # Apply range filters, if needed
        # if "min_range" in self.config and "max_range" in self.config:
        #     valid_idx = np.logical_and(
        #         np.linalg.norm(points, axis=1) >= self.config["min_range"],
        #         np.linalg.norm(points, axis=1) <= self.config["max_range"]
        #     )
        #     points = points[valid_idx]

        # # Apply pose transform, if desired
        # if self.config.get("apply_pose", False):
        tmp_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        tmp_cloud.transform(pose)
        points = np.asarray(tmp_cloud.points)

        return points, pose