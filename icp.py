import numpy as np
from scipy.spatial import KDTree


def best_fit_transform(A, B):
    assert A.shape == B.shape

    # Step 1: Compute centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Step 2: Center the points
    AA = A - centroid_A
    BB = B - centroid_B

    # Step 3: Compute rotation matrix using SVD
    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Special reflection case
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    # Step 4: Compute translation
    t = centroid_B - R @ centroid_A

    return R, t


def icp(A, B, max_iterations=50, tolerance=5e-6):
    src = np.copy(A)
    prev_error = float("inf")
    errors = []

    for i in range(max_iterations):
        # Step 1: Find closest points
        tree = KDTree(B)
        distances, indices = tree.query(src)
        closest_B = B[indices]

        # Step 2: Compute transformation
        R, t = best_fit_transform(src, closest_B)

        # Step 3: Apply transformation
        src = (R @ src.T).T + t

        # Step 4: Compute error
        mean_error = np.mean(distances)
        errors.append(mean_error)

        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # Final transformation
    R_final, t_final = best_fit_transform(A, src)
    return R_final, t_final, src, errors
