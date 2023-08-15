# --- Import
import os
import numpy as np


# --- Raw Point Cloud ---
class RawPointCloud:
    # --- Init Raw Point Cloud
    def __init__(self, pc_path, max_frames=False, num_max_frames=0):
        self.pc_path = pc_path
        self.max_frames = max_frames
        self.num_max_frames = num_max_frames
        self.raw_point_cloud = []
        self.num_frames = 0

        self.load_raw_point_cloud()

    # --- Load Raw Point Cloud (Retrieve Raw Point Cloud from .mf4 File)
    def load_raw_point_cloud(self):
        pc_files = os.listdir(self.pc_path)
        self.num_frames = len(pc_files)
        print(f'Number of Frames: {self.num_frames}')

        for frame_idx, pc_file in enumerate(pc_files):
            file_path = self.pc_path + r"\\" + pc_file
            data_frame = np.fromfile(file_path, sep=' ')
            data_frame = np.reshape(data_frame, (-1, 4))
            num_points = len(data_frame)
            print(f'Frame {frame_idx + 1}/{self.num_frames}')
            print(f'Number of Points: {num_points}')
            raw_pc_frame = RawPointCloudFrame(data_frame, frame_idx, num_points, self.num_frames)
            self.raw_point_cloud.append(raw_pc_frame)
            if self.max_frames and frame_idx == (self.num_max_frames - 1):
                break


# --- Raw Point Cloud Frame---
class RawPointCloudFrame(RawPointCloud):
    # --- Init Raw Point Cloud Frame
    def __init__(self, data_frame, frame_idx=0, num_points=0, num_frames=0):
        self.frame_idx = frame_idx
        self.num_points = num_points
        self.num_frames = num_frames
        self.x = list()
        self.y = list()
        self.z = list()
        self.ref = list()
        self.pcdXYZ = None
        self.pcdXYZref = None

        self.read_frame(data_frame)

    # --- Read Point Cloud Data from Frame
    def read_frame(self, data_frame):
        for pointCloudData in data_frame:                                                                               #  x, y, z, reflectivity
            self.x.append(pointCloudData[0])
            self.y.append(pointCloudData[1])
            self.z.append(pointCloudData[2])
            self.ref.append(pointCloudData[3])

        self.x = np.vstack(self.x).astype(np.float32)
        self.y = np.vstack(self.y).astype(np.float32)
        self.z = np.vstack(self.z).astype(np.float32)
        self.ref = np.vstack(self.ref).astype(np.float32)
        self.pcdXYZ = np.hstack((self.x, self.y, self.z))
        self.pcdXYZref = np.hstack((self.x, self.y, self.z, self.ref))
