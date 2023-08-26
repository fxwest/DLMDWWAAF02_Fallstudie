# -------------------------------
# ----------- Import ------------
# -------------------------------
import PointCloud as pc
import PointCloudVisu as pcv


# -------------------------------
# -------- Hyperparameters ------
# -------------------------------
NUM_MAX_FRAMES = 5
PCD_FOLDER = r"C:\Users\Q554273\OneDrive - BMW Group\Selbststudium\_Master\LokalisierungBewegungsplanungFusion\Fallstudie\Data\2011_09_26_drive_0052_extract\2011_09_26\2011_09_26_drive_0052_extract\velodyne_points\data"


# -------------------------------
# -------- Main Function --------
# -------------------------------
def main():
    raw_pc_trace = pc.PointCloudTrace(PCD_FOLDER, NUM_MAX_FRAMES)
    lidar_viewer = pcv.LidarViewer(raw_pc_trace.raw_pc_frame_list, raw_pc_trace.num_frames, raw_pc_trace.num_max_frames)


# -------------------------------
# ----- Call Main Function ------
# -------------------------------
if __name__ == '__main__':
    main()
