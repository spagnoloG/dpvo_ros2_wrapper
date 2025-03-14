# dpvo_ros2_wrapper

A ROS2 wrapper for DPVO (Deep Patch Visual Odometry/SLAM) that simplifies integration of DPVO outputs with ROS2-based systems. This wrapper subscribes to DPVO’s outputs and publishes:
- A TF transform for the current drone/camera pose.
- The full trajectory as a MarkerArray (line strip).
- Camera frustum markers (one per pose) similar to the DPVO viewer.
- A point cloud (sensor_msgs/PointCloud2) built from DPVO point/color data.
- An image stream (sensor_msgs/Image) from the DPVO pipeline.

All data are published in the `map` frame.

---

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
  - [System Dependencies](#system-dependencies)
  - [DPVO Setup in a Micromamba Environment](#dpvo-setup-in-a-micromamba-environment)
  - [DPVO Package Installation](#dpvo-package-installation)
  - [CMake 3.31 Installation](#cmake-331-installation)
  - [Pangolin Installation (Optional)](#pangolin-installation-optional)
  - [Wrapper Setup](#wrapper-setup)
- [Exposed Topics](#exposed-topics)
- [Usage](#usage)
  - [Running the ROS2 Publisher Node](#running-the-ros2-publisher-node)
  - [Data Flow and Topic Details](#data-flow-and-topic-details)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## Overview

The `dpvo_ros2_wrapper` bridges DPVO with ROS2 by launching a Python-based node that wraps the DPVO pipeline. The node:
- Reads images (or video) and calibration files,
- Runs DPVO to obtain pose, point cloud, and image data,
- Publishes the current pose as a TF transform,
- Visualizes the trajectory and camera frustums as MarkerArrays,
- Publishes a point cloud and image stream for real-time visualization in RViz.

---

## Installation

### System Dependencies

Ensure your system meets these prerequisites:
- Ubuntu 20.04/22.04
- CUDA 12
- Python 3.8+ (with pip)
- ROS2 (Foxy, Humble, or Rolling)
- Standard build tools (e.g., gcc, cmake)

Install essential packages:
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libopencv-dev pkg-config
```

### DPVO Setup in a Micromamba Environment

1. **Install Micromamba** (if not already installed):  
   Follow the instructions on the [Micromamba GitHub repository](https://github.com/mamba-org/micromamba).

2. **Clone the DPVO Repository**:
   ```bash
   cd ~/Documents
   git clone https://github.com/princeton-vl/DPVO.git --recursive
   cd DPVO
   ```

3. **Create and Activate the DPVO Environment**:  
   Use the provided `environment.yml` with micromamba.
   ```bash
   micromamba create --name dpvo --file environment.yml
   micromamba activate dpvo
   pip install transforms3d
   ```

### DPVO Package Installation

1. **Prepare Dependencies**:  
   DPVO requires Eigen. Download and unzip it:
   ```bash
   wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
   unzip eigen-3.4.0.zip -d thirdparty
   ```

2. **Install DPVO**:
   ```bash
   pip install .
   ```

3. **Download Pretrained Models and Data** (~2GB):
   ```bash
   ./download_models_and_data.sh
   ```

### CMake 3.31 Installation

1. **Remove Old CMake (Optional)**
   ```bash
   sudo apt remove --purge cmake
   ```

2. **Download and Extract CMake 3.31**
   ```bash
   wget https://github.com/Kitware/CMake/releases/download/v3.31.0/cmake-3.31.0.tar.gz
   tar -xvzf cmake-3.31.0.tar.gz
   cd cmake-3.31.0
   ```

3. **Build and Install**
   ```bash
   ./bootstrap
   make -j$(nproc)
   sudo make install
   ```

### Pangolin Installation (Optional)

For real-time visualization via the DPVO viewer:
1. **Install Pangolin prerequisites and build Pangolin**:
   ```bash
   ./Pangolin/scripts/install_prerequisites.sh recommended
   mkdir -p Pangolin/build && cd Pangolin/build
   cmake ..
   make -j8
   sudo make install
   cd ../..
   ```
2. **Install the DPVO Viewer**:
   ```bash
   pip install ./DPViewer
   ```

### Wrapper Setup

Clone the wrapper repository and build the workspace:
```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/spagnoloG/dpvo_ros2_wrapper.git

colcon build
```

---

## Exposed Topics

The `dpvo_ros2_wrapper` node publishes several ROS2 topics to facilitate integration with other ROS2 applications.

### Published Topics

- **`/trajectory_markers`**  
  - **Message Type:** `visualization_msgs/msg/MarkerArray`  
  - **Description:** Publishes the full trajectory as a line strip marker, visualizing the path of the camera/drone.

- **`/camera_frustums`**  
  - **Message Type:** `visualization_msgs/msg/MarkerArray`  
  - **Description:** Publishes camera frustum markers (one per pose) that mimic the DPVO viewer visualization.

- **`/point_cloud`**  
  - **Message Type:** `sensor_msgs/msg/PointCloud2`  
  - **Description:** Publishes a point cloud built from DPVO point/color data for 3D visualization.

- **`/image_stream`**  
  - **Message Type:** `sensor_msgs/msg/Image`  
  - **Description:** Publishes an image stream output from the DPVO pipeline.

### TF Transforms

- **TF Broadcast:**  
  - **Frames:** A transform is broadcast from the `map` frame to the `drone` frame using `tf2_ros/TransformBroadcaster`, representing the current pose of the camera/drone.

---

## Usage

### Running the ROS2 Publisher Node

After building the workspace, source the setup file and launch the node:
```bash
cd ~/colcon_ws
source install/setup.bash
ros2 launch dpvo_ros2_wrapper dpvo_ros2_wrapper_launch.py
```

### Data Flow and Topic Details

- **Input Data:**  
  The wrapper takes an image directory (or video file), along with calibration, network, and configuration files as input parameters.

- **Processing:**  
  The DPVO pipeline processes the input data to compute the current pose, trajectory, point cloud, and image stream.

- **Output Data:**  
  The processed data is published on the topics listed above:
  - The **TF transform** provides the current pose of the camera/drone.
  - **`/trajectory_markers`** visualizes the full trajectory.
  - **`/camera_frustums`** displays individual camera frustums for each pose.
  - **`/point_cloud`** and **`/image_stream`** offer detailed visualization in RViz.

To disable the Pangolin viewer (if real-time DPVO visualization is not required), set the `--viz` flag to `false` when launching:
```bash
ros2 launch dpvo_ros2_wrapper dpvo_ros2_wrapper_launch.py --ros-args -p viz:=false
```

---

## Contributing

Contributions are welcome! Please fork the repository, open issues, or submit pull requests with improvements or bug fixes.

---

## License

This project is licensed under the GNU General Public License v3 (GPLv3). See the [LICENSE](LICENSE) file for details.

---

## Acknowledgements

- [DPVO](https://github.com/princeton-vl/DPVO) for the core Deep Patch Visual Odometry/SLAM implementation.