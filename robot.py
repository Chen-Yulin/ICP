import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# -----------------------
# Helper Functions
# -----------------------


def generate_scene():
    wall1 = np.array([[x, 0] for x in np.linspace(0, 5, 50)])
    wall2 = np.array([[5, y] for y in np.linspace(0, 5, 50)])
    wall3 = np.array([[x, 5] for x in np.linspace(5, 0, 50)])
    wall4 = np.array([[0, y] for y in np.linspace(5, 0, 50)])
    table = np.array(
        [[x, y] for x in np.linspace(2, 3, 10) for y in np.linspace(2, 3, 10)]
    )
    cabinet = np.array(
        [[x, y] for x in np.linspace(4, 5, 10) for y in np.linspace(0, 3, 30)]
    )
    shelf = np.array(
        [[x, y] for x in np.linspace(1, 4, 30) for y in np.linspace(4.5, 5, 5)]
    )

    # Combine all
    scene = np.vstack((wall1, wall2, wall3, wall4, table, cabinet, shelf))
    return scene


def apply_transform(points, R, t):
    return (R @ points.T).T + t


def best_fit_transform(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    return R, t


def icp(A, B, max_iterations=30, tolerance=1e-6):
    src = np.copy(A)
    prev_error = float("inf")
    for i in range(max_iterations):
        tree = KDTree(B)
        distances, indices = tree.query(src)
        B_matched = B[indices]
        R, t = best_fit_transform(src, B_matched)
        src = apply_transform(src, R, t)
        mean_error = np.mean(distances)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    R_final, t_final = best_fit_transform(A, src)
    return R_final, t_final


def transform_from_pose(pose):
    x, y, theta = pose
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    t = np.array([x, y])
    return R, t


def draw_robot_orientation(ax, pose, length=0.2, color="k"):
    x, y, theta = pose
    dx = length * np.cos(theta)
    dy = length * np.sin(theta)
    ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc=color, ec=color)

def extract_view(scene, robot_pose, view_radius=5.0):
    """Simulate sensor view by extracting scene points within radius"""
    R, t = transform_from_pose(robot_pose)
    transformed_scene = apply_transform(scene, np.linalg.inv(R), -R.T @ t)
    local = transformed_scene[np.linalg.norm(transformed_scene, axis=1) < view_radius]
    return local


# -----------------------
# Simulation Parameters
# -----------------------

scene = generate_scene()

# Ground truth robot path (x, y, theta)
path_gt = [
    [1.0, 1.0, np.radians(0)],
    [1.5, 0.8, np.radians(0)],
    [2.0, 1.0, np.radians(10)],
    [3.0, 1.5, np.radians(15)],
    [3.5, 2.5, np.radians(30)],
    [3.2, 3.8, np.radians(45)],
    [2.0, 4.2, np.radians(70)],
]

# -----------------------
# Reconstruct Path via ICP
# -----------------------

trajectory_est = [[0.0, 0.0, 0.0]]  # Start from origin
pose_est = np.eye(3)

prev_scan = extract_view(scene, path_gt[0])

for i in range(1, len(path_gt)):
    cur_scan = extract_view(scene, path_gt[i])
    R_icp, t_icp = icp(cur_scan, prev_scan)

    # Compose transformation
    T = np.eye(3)
    T[:2, :2] = R_icp
    T[:2, 2] = t_icp
    pose_est = pose_est @ T

    # Extract estimated position
    x, y = pose_est[:2, 2]
    theta = np.arctan2(pose_est[1, 0], pose_est[0, 0])
    trajectory_est.append([x, y, theta])

    prev_scan = cur_scan

# -----------------------
# Visualization
# -----------------------

# Convert to arrays
path_gt_np = np.array([[p[0], p[1]] for p in path_gt])
path_est_np = np.array(
    [
        [
            p[0] + path_gt_np[0][0],
            p[1] + path_gt_np[0][1],
        ]
        for p in trajectory_est
    ]
)

plt.figure(figsize=(8, 8))
plt.scatter(scene[:, 0], scene[:, 1], s=3, c="blue", label="Scene")
for pose in path_gt:
    draw_robot_orientation(plt.gca(), pose, color="g")

for i, pose_est in enumerate(trajectory_est):
    # Adjust the estimated pose by the initial ground truth position for visualization
    adjusted_pose = [
        pose_est[0] + path_gt[0][0],
        pose_est[1] + path_gt[0][1],
        pose_est[2],
    ]
    draw_robot_orientation(plt.gca(), adjusted_pose, color="r")

plt.plot(path_gt_np[:, 0], path_gt_np[:, 1], "g-o", label="Ground Truth Path")
plt.plot(path_est_np[:, 0], path_est_np[:, 1], "r--o", label="Estimated Path (ICP)")
plt.legend()
plt.axis("equal")
plt.title("Robot Path Reconstruction via ICP")
plt.grid(False)
plt.show()
