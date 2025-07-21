# ICP-based Robot Localization

This project demonstrates the use of the Iterative Closest Point (ICP) algorithm for robot localization and path reconstruction in a simulated 2D environment.

## Description

The `icp.py` script implements the core ICP algorithm, which is used to find the optimal transformation between two sets of points. The `robot.py` script simulates a robot moving through a predefined scene, taking sensor scans (point clouds) at different poses. These scans are then used by the ICP algorithm to reconstruct the robot's trajectory.

## Files

-   `icp.py`: Contains the implementation of the ICP algorithm (`best_fit_transform` and `icp` functions).
-   `test.py`: Contains a basic test case for the ICP algorithm, demonstrating its ability to align two point clouds.
-   `robot.py`: Simulates the robot's movement, generates sensor scans, applies ICP for localization, and visualizes the results.

## How to Run

To run the robot localization simulation, execute the `robot.py` script:

```bash
python robot.py
```

To run the ICP test case, execute the `test.py` script:

```bash
python test.py
```

## Visualization

The `test.py` script generates two plots:

-   **Before ICP Alignment**: Shows the original and transformed point clouds before alignment.
-   **After ICP Alignment**: Shows the point clouds after ICP has aligned them.
-   **ICP Alignment Error Over Iterations**: Shows the convergence of the ICP algorithm.

The `robot.py` script generates a plot showing:

-   The static scene (blue dots).
-   The ground truth robot path (green line with circles) and its orientation (green arrows).
-   The estimated robot path reconstructed using ICP (red dashed line with circles) and its orientation (red arrows).
