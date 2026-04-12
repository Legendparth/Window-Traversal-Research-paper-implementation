# Window Traversal Path Planning for Quadrotors

This repository contains the ongoing implementation of the window traversal path planning algorithm, inspired by the research paper by **Ashwini Ratoo and Midhun E K** on *Quadrotor Guidance for Window Traversal: A Bearings-Only Approach*. 

This project implements an autonomous mode where a quadrotor identifies a window, calculates its relative pose, and smoothly follows an elliptical trajectory to traverse through it safely.

## Implementation Details

The core logic is implemented in `imav_indoor_2026/window_traversal.py`. The pipeline can be broken down into the following key steps:

### 1. Identifying Window Corners in NED Frame
Before the drone can execute the path planning algorithm, it must determine the 3D coordinates of the four window corners ($E_1, E_2, E_3, E_4$) in the local NED (North-East-Down) frame.
* **Vision Data:** The drone leverages an RGB camera masked to find the target window (by color) and extracts bounding box corners using OpenCV contours.
* **Depth Mapping:** The RGB pixel coordinates are mapped to a Depth camera to retrieve the range/distance to each corner ($R_{ei}$).
* **Body Frame Coordinates:** Using the ranges and the camera intrinsics (bearing angles $\theta_x, \theta_y$), the corners are projected into the drone's Body Frame.
* **Transformation to NED:** Using the drone's current pose (from `/fmu/out/vehicle_local_position_v1` and `/fmu/out/vehicle_attitude`), the coordinates are rotated and translated into the global NED frame. The conditions of the window being planar and rectangular undergo cross-product and dot-product checks to validate the lock.

### 2. Transitioning to the Research Paper's Target World Frame
The research paper defines a specific target frame (World Frame) based on the window geometry to simplify the elliptical path calculations.
* The unit vectors for the window's World Frame are established:
  * **$X_w$:** Along the bottom edge of the window (e.g., $E_2 - E_1$).
  * **$Z_w$:** Along the left edge of the window (e.g., $E_1 - E_4$).
  * **$Y_w$:** Orthogonal to the window plane ($\vec{Z_w} \times \vec{X_w}$).
* The drone's relative position vectors to the window corners are projected onto these $X_w, Y_w, Z_w$ axes, converting the state variables entirely into the paper's expected frame representations.

### 3. Elliptical Path Planning Algorithm
Once the positions are formulated in the target World Frame, the drone calculates an intercepting trajectory based on the paper's elliptical path model.
* **Path Generation:** The path uses bisecting bearing angles ($\alpha, \beta, \gamma, \chi$) to ensure the drone follows an elliptical curve towards the window center rather than moving in a straight line, mitigating collision risks with the window frame.
* **Velocity Control:** The target moving directions ($v_{xw}, v_{yw}, v_{zw}$) are generated in the World Frame.
* **Execution:** These target velocities are transformed back into the NED frame ($V_{ned}$) and sent to the flight controller via `TrajectorySetpoint` messages inside PX4 Offboard mode.

## Mathematical Formulations

To translate raw sensor readings into autonomous actions, the following mathematical formulations are used across the algorithm:

### Window Corner Projection (Body Frame)
Given a depth reading $R$ and camera bearing angles $\theta_x$ and $\theta_y$, the corner coordinates in the Body Frame ($[x_b, y_b, z_b]$) are calculated as:

$$x_b = R$$
$$y_b = R \cdot \tan(\theta_x)$$
$$z_b = -R \cdot \tan(\theta_y)$$

These values represent the Forward-Right-Down standard frame, taking into account any physical camera offsets.

### Quaternion Rotation to NED Frame
To accurately map the corners into the drone's global NED frame, a quaternion rotation is performed using the helper class `Quaternions`. A vector in the body frame $v_b$ is rotated to the NED frame $v_{ned}$ using the drone's orientation quaternion $\mathbf{q}$:

$$\mathbf{\hat{v}}_{ned} = \mathbf{q} \otimes [0, \mathbf{v}_b] \otimes \mathbf{q}^{-1}$$
$$\text{Target Position} = \mathbf{\hat{v}}_{ned} + \text{Drone Position}$$

### Frame Validation
Before committing to the window target, the vectors formed by the window edges are verified to check if they form a valid plane and rectangle shape using dot and cross products:

$$\text{Cross Products Check: } \mathbf{v}_{1,2} \times \mathbf{v}_{3,4} \approx 0$$
$$\text{Dot Products Check: } \mathbf{v}_{1,2} \cdot \mathbf{v}_{2,3} \approx 0$$

### Bearing Angles and Trajectory Velocities
The positions of the window corners projected onto the target World Frame are used to calculate the elevation ($\alpha$) and azimuth ($\beta$) angles:

$$\alpha = \arcsin\left(\frac{p_z}{\text{distance}}\right)$$
$$\beta = \arctan\left(\frac{p_y}{p_x}\right)$$

From these, bisector angles and elliptical offsets ($S_\gamma$, $S_\chi$) determine the desired path angles $\gamma_{des}$ and $\chi_{des}$, driving the velocity references:

$$v_{xw} = V_{\text{traverse}} \cdot \cos(\gamma_{des}) \cdot \cos(\chi_{des})$$
$$v_{yw} = V_{\text{traverse}} \cdot \cos(\gamma_{des}) \cdot \sin(\chi_{des})$$
$$v_{zw} = V_{\text{traverse}} \cdot \sin(\gamma_{des})$$

## Key Features
* **PX4 Offboard Control Interface:** Uses `px4_msgs` over micro-ROS/DDS to command `TrajectorySetpoint`.
* **Hybrid Control Mode:** Supports complete manual flight via a Joystick (`sensor_msgs/Joy`) and includes an override button to trigger the autonomous "Window Traversal Mode".
* **Real-Time Visualization:** Renders the RGB view and calculated Normalized Depth View via OpenCV to visualize corner acquisitions and tracking.

## Dependencies
* ROS2 Humble
* PX4 Autopilot (`px4_msgs`)
* OpenCV (`cv_bridge`)
* NumPy
