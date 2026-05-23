# Formula Student Driverless (FSD) Autonomous Racing Stack

> **A complete ROS 2 autonomous racing pipeline featuring YOLOv8 perception, GraphSLAM mapping, and Model Predictive Control (MPC) trajectory optimization.**

## About the Project
This repository contains the software stack developed for my master's thesis on autonomous racing path planning under Formula Student rules. The inspiration behind this topic stems from my time as a member of the **Centaurus Racing Team** at the **University of Thessaly** (Volos, Greece). The system architecture, mathematical implementation and code within this repository represent my own independent research and development.

The system is designed to drive an autonomous vehicle through an unknown track using a dual-phase approach: an initial **Exploration Lap** to safely map the environment, followed by an **Optimization Lap** that computes and executes the absolute physical limits of the vehicle.

---

## System Architecture
The pipeline is built on **ROS 2** and tested in the **CARLA Simulator**. The autonomous navigation relies on a core sensor suite consisting of an **RGB Camera** and a **180° LiDAR**. The software stack is divided into four main subsystems:

### 1. Perception
+ **Camera Detection:** Utilizes YOLOv8 (trained offline on the FSOCO dataset) to detect and classify cones (Yellow, Blue, Orange) in 2D image space.
+ **LiDAR Perception:** Process raw point clouds using PCL (PassThrough filtering, RANSAC ground removal and Euclidean Clustering) to extract 3D cone centroids.
+ **Sensor Fusion:** Projects 3D LiDAR centroids onto the 2D camera plane using the pinhole camera model, assigning YOLO color classifications to the physical LiDAR points.

### 2. Localization and Mapping
+ **Extended Kalman Filter (EKF):** Fuses high-frequency IMU data with low-frequency GNSS data
+ **GraphSLAM:** A custom 2D Pose Graph SLAM engine built from scratch using Eigen and Cholesky decomposition (SimplicialLDLT). It tracks cone color probabilities using a recursive Bayes filter and optimizes the track map upon loop closure.

### 3. Path Planning
+ **Phase 1 - Exploration (Lap 1):** Uses Bowyer-Watson Delaunay Triangulation to extract safe track boundaries and a drivable centerline. The centerline is smoothed using Uniform Cubic B-Splines.
+ **Phase 2 - Optimization (Lap 2+):** An Elastic Band Line Optimizer pulls the trajectory toward the apex of corners. A Velocity Profiler calculates absolute speed limits based on Menger curvature and tire friction circles.

### 4. Control
+ **Exploration Control:** Geometric Pure Pursuit for lateral control and a PID controller for longitudinal speed tracking.
+ **Optimization Control (MPC):** An OSQP-Eigen driven Model Predictive Controller utilizing a kinematic bicycle model. It strictly enforces physical actuator limits (steering angle, max acceleration/deceleration) while following the optimal speed profile.

---

## Repository Structure
```txt
autonomous-racing-fsd/
├── carla_setup/                  # CARLA sensor configs and bridge launch scripts
│   ├── objects.json
│   ├── run_bridge.sh
│   └── unfreeze_carla.py
├── offline_training/             # YOLOv8 dataset prep, training, and weights
│   ├── dataset_tools/
│   ├── weights/                  # Contains best.pt
│   └── train.py
└── src/                          # ROS 2 Workspace
    ├── bringup/                  # Master launch files
    ├── camera_detection/         # YOLOv8 ROS 2 node
    ├── interfaces/               # Custom Cone and ConeArray messages
    ├── lidar_perception/         # PCL clustering node
    ├── localization/             # EKF, GraphSLAM, and RViz vizualization
    ├── path_planning/            # State Machine, Delaunay, MPC, Optimizers
    └── sensor_fusion/            # Camera/LiDAR projection node
```

---

## Prerequisites & Dependencies

To build and run this stack, you need the following environment:
+ **OS:** Ubuntu 22.04
+ **ROS 2**: Humble
+ **Simulator:** CARLA (Version 0.9.14+) & `carla-ros-bridge`
+ **Python Libraries:** `ultralytics`, `scipy`, `numpy`, `opencv-python`
+ **C++ Libraries:** Eigen3, PCL

### Manual Installation Required: OsqpEigen

The Model Predictive Controller relies on the `OsqpEigen` library, which cannot be installed automatically via ROS `rosdep`. You must build it from source before compiling this workspace.

For full documentation and troubleshooting, visit the official repository: [gbionics/osqp-eigen](https://github.com/gbionics/osqp-eigen.git).

Run the following commands outside the ROS workspace to install it:

```bash
git clone https://github.com/gbionics/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=<custom-folder> ../
make
make install
```

---

## Configuration Notes (Read Before Running)
Because this is an active research repository, certain file paths are currently hardcoded to a specific local machine setup. **You must update these paths in the the source code before building the project.**

For instance:
1. **YOLO Weights Path:** In `src/camera_detection/camera_detector_node.py`, update `MODEL_PATH` to point your local `best.pt` file.
2. **Telemetry CSV Paths:** In `src/path_planning/src/state_machine_node.cpp`, update the string variables `filename` and the `telemetry_file.open()` path to point to valid directories on your machine where the node has write permissions.

Moreover, every ROS 2 topic names that are used, are hardcoded into the local machine setup. You must update **ALL** the topics names before trying to run the project, otherwise the appropriate message will not be received or sent.

---

## Instalation & Usage

1. Clone the repository:
```bash
git clone
cd autonomous-racing-fsd
```

2. Build the workspace: Use the provided bash script to automatically resolve ROS 2 dependencies and build the packages.
```bash
chmod +x setup.sh
./setup.sh
```

3. Launch the Stack: You will need three terminals

**Terminal 1:** Launch CARLA Simulator and press play on the appropriate map.

**Terminal 2:** Launch the carla-ros-bridge. First copy the three files inside the `carla_setup/` folder (`objects.json`, `run_bridge.sh` and `unfreeze_carla.py`) directly into the root folder of your `carla-ros-bridge` installation. Then run the bridge from there:
```bash
cd /path/to/your/carla-ros-bridge
chmod +x run_bridge.sh
./run_bridge.sh
```

**Terminal 3:** Launch the FSD ROS 2 pipeline.
```bash
source install/setup.bash
ros2 launch bringup simulation.launch.py
```

### Shutting Down the Simulation

To safely stop the simulation without freezing the CARLA server, follow this exact order:

1. **Terminal 3 (FSD Pipeline):** Press `Ctrl + C` to stop the autonomous nodes.
2. **Terminal 2 (ROS Bridge):** Press `Ctrl + C` to trigger the `unfreeze_carla.py` to return the simulation to asynchronous mode.
3. **Terminal 1 (CARLA):** Open the CARLA simulator and press stop to the simulation or press `Ctrl + C` to shut down the simulator.

---

## License
This project is licensed under the Apache 2.0 License. See the `LICENSE` file for details.


