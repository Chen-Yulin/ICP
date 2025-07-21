import numpy as np
import matplotlib.pyplot as plt


def generate_scene():
    # Room boundaries (square)
    wall1 = np.array([[x, 0] for x in np.linspace(0, 5, 50)])
    wall2 = np.array([[5, y] for y in np.linspace(0, 5, 50)])
    wall3 = np.array([[x, 5] for x in np.linspace(5, 0, 50)])
    wall4 = np.array([[0, y] for y in np.linspace(5, 0, 50)])

    # Table
    table = np.array(
        [[x, y] for x in np.linspace(1.5, 3, 15) for y in np.linspace(2, 3, 10)]
    )
    cabinet = np.array(
        [[x, y] for x in np.linspace(4, 5, 10) for y in np.linspace(0, 3, 30)]
    )

    # Combine all
    scene = np.vstack((wall1, wall2, wall3, wall4, table, cabinet))
    return scene


def apply_transformation(points, R, t):
    return (R @ points.T).T + t


# Generate original point cloud (scene)
A = generate_scene()

# Define transformation
angle = np.radians(20)
R_true = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
t_true = np.array([1.0, 0.5])
B = apply_transformation(A, R_true, t_true)

# Apply ICP
from scipy.spatial import KDTree


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


def icp(A, B, max_iterations=100, tolerance=1e-6):
    src = np.copy(A)
    prev_error = float("inf")
    errors = []

    for i in range(max_iterations):
        tree = KDTree(B)
        distances, indices = tree.query(src)
        B_matched = B[indices]
        R, t = best_fit_transform(src, B_matched)
        src = (R @ src.T).T + t
        mean_error = np.mean(distances)
        errors.append(mean_error)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    R_final, t_final = best_fit_transform(A, src)
    return R_final, t_final, src, errors


# Run ICP
R_icp, t_icp, A_aligned, errors = icp(A, B)

# Plot results
fig, ax = plt.subplots(1, 2, figsize=(14, 6))

# Before alignment
ax[0].scatter(A[:, 0], A[:, 1], label="Original A", s=5)
ax[0].scatter(B[:, 0], B[:, 1], label="Transformed B", s=5)
ax[0].set_title("Before ICP Alignment")
ax[0].legend()
ax[0].axis("equal")

# After alignment
ax[1].scatter(A_aligned[:, 0], A_aligned[:, 1], label="Aligned A", s=5)
ax[1].scatter(B[:, 0], B[:, 1], label="Target B", s=5)
ax[1].set_title("After ICP Alignment")
ax[1].legend()
ax[1].axis("equal")

plt.tight_layout()
plt.show()

# Plot error convergence
plt.figure()
plt.plot(errors, marker="o")
plt.title("ICP Alignment Error Over Iterations")
plt.xlabel("Iteration")
plt.ylabel("Mean Error")
plt.grid(True)
plt.show()
