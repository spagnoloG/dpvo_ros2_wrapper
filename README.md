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
  - [Pangolin Installation (Optional)](#pangolin-installation-optional)
- [Usage](#usage)
  - [Running the ROS2 Publisher Node](#running-the-ros2-publisher-node)
  - [Exposed Topics](#exposed-topics)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## Overview

The `dpvo_ros2_wrapper` bridges DPVO with ROS2. It launches a Python-based ROS2 node that wraps the DPVO pipeline. The node:
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

### CMake 3.31 version installation

1. Remove Old CMake (Optional)
   ```bash
   sudo apt remove --purge cmake
   ```

2. Download and Extract CMake 3.31
   ```bash
   wget https://github.com/Kitware/CMake/releases/download/v3.31.0/cmake-3.31.0.tar.gz
   tar -xvzf cmake-3.31.0.tar.gz
   cd cmake-3.31.0
   ```

3. Build and Install
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

### Wrapper setup

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/spagnoloG/dpvo_ros2_wrapper.git

colcon build
```

---

## Usage


```bash
micromamba activate dpvo
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