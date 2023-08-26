# -------------------------------
# ----------- Import ------------
# -------------------------------
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN


# -------------------------------
# -------- Hyperparameter -------
# -------------------------------
GROUND_COLOR = [0.8, 0.8, 0.8]
OUTLIER_COLOR = [1.0, 0, 0]
NOISE_COLOR = [0.5, 0.5, 0.5]


# -------------------------------
# ---------- TRIM FOV -----------
# -------------------------------
def trim_fov(pc_frames, x_axis, y_axis, z_axis):
    print(f"Starting FOV Trimming")
    trimmed_pc_frames = []
    for frame in pc_frames:
        trimmed_pc = []
        pc_array = np.asarray(frame.pcdXYZ.points)
        for point in pc_array:
            if point[0] < x_axis[1] and point[0] > x_axis[0]:             # Filter X
                if point[1] < y_axis[1] and point[1] > y_axis[0]:         # Filter Y
                    if point[2] < z_axis[1] and point[2] > z_axis[0]:     # Filter Z
                        trimmed_pc.append(point)
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(trimmed_pc)
        trimmed_pc_frames.append(pc)
    return trimmed_pc_frames


# -------------------------------
# ----------- RANSAC ------------
# -------------------------------
def get_ground_plane_ransac(pc_frames, distance_threshold=0.01, ransac_n=3, num_iterations=1000):
    ground_pc_frame_list = []
    outlier_pc_frame_list = []
    print(f"Starting Ground Segmentation with RANSAC")
    for frame in pc_frames:
        # --- Run RANSAC
        plane_model, inliers = frame.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
        ground_pc = frame.select_by_index(inliers)
        outlier_pc = frame.select_by_index(inliers, invert=True)

        # --- Segment Color
        ground_pc.paint_uniform_color(GROUND_COLOR)
        outlier_pc.paint_uniform_color(OUTLIER_COLOR)

        ground_pc_frame_list.append(ground_pc)
        outlier_pc_frame_list.append(outlier_pc)

    return ground_pc_frame_list, outlier_pc_frame_list


def get_clusters_dbscan(pc_frames, eps=0.1, min_points=10):
    print(f"Starting Clustering with DBSCAN")
    clustered_pc_frames = pc_frames
    for idx, frame in enumerate(clustered_pc_frames):
        pc_array = np.asarray(frame.points)
        dbscan = DBSCAN(eps=eps, min_samples=min_points)
        labels = dbscan.fit_predict(pc_array)

        max_label = labels.max()
        print(f"Frame {idx +1} has {max_label + 1} Clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        filtered_colors = colors.copy()
        for idx, point in enumerate(colors[labels < 0]):
            point[:3] = NOISE_COLOR
            filtered_colors[idx] = point
        frame.colors = o3d.utility.Vector3dVector(filtered_colors[:, :3])

    return clustered_pc_frames
