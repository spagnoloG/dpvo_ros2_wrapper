#!/usr/bin/env python3

"""
DPVO Combined ROS2 Wrapper Node

This node subscribes to the outputs of DPVO and publishes:
  1. A TF transform for the current drone/camera pose.
  2. The full trajectory as a MarkerArray (line strip).
  3. Camera frustum markers (one per pose) that mimic the DPVO viewer.
  4. A point cloud (sensor_msgs/PointCloud2) built from DPVO point/color data.
  5. An image stream (sensor_msgs/Image) from the DPVO pipeline.

All data are published in the "map" frame.
"""

### SOLVE ENVIRONMENT ISSUES ###
import sys
import os


def add_env_to_sys_path(env_name, python_version="3.10"):
    """
    Searches for an environment (Anaconda, Miniconda, or Micromamba) with the given env_name and
    prepends its site-packages directory to sys.path.

    Parameters:
      env_name (str): The name of the environment (e.g., "dpvo")
      python_version (str): Python version used in the environment (default "3.10")

    Returns:
      bool: True if a matching site-packages directory was found and added, False otherwise.
    """
    home = os.path.expanduser("~")
    possible_paths = [
        os.path.join(
            home,
            "anaconda3",
            "envs",
            env_name,
            "lib",
            f"python{python_version}",
            "site-packages",
        ),
        os.path.join(
            home,
            "miniconda3",
            "envs",
            env_name,
            "lib",
            f"python{python_version}",
            "site-packages",
        ),
        os.path.join(
            home,
            "micromamba",
            "envs",
            env_name,
            "lib",
            f"python{python_version}",
            "site-packages",
        ),
    ]

    for path in possible_paths:
        if os.path.exists(path):
            if path not in sys.path:
                sys.path.insert(0, path)
            return True

    print(f"Warning: Could not find site-packages for environment '{env_name}'")
    return False


if not add_env_to_sys_path("dpvo"):
    print(
        "Failed to add the dpvo environment's site-packages to sys.path. Are you sure the environment exists?"
    )
else:
    print("Successfully added the dpvo environment's site-packages to sys.path")

##

import os
from pathlib import Path
import cv2
from evo.core.trajectory import PoseTrajectory3D

import threading
from multiprocessing import Process, Queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField, Image
import struct
from cv_bridge import CvBridge

# Check if ~/Documents/DPVO exists, fail otherwise
if not os.path.exists(os.path.expanduser("~/Documents/DPVO")):
    raise FileNotFoundError("Directory ~/Documents/DPVO not found")
else:
    sys.path.insert(0, os.path.expanduser("~/Documents/DPVO"))

import numpy as np
import torch
from dpvo.config import cfg
from dpvo.dpvo import DPVO
from dpvo.stream import image_stream, video_stream
from dpvo.utils import Timer
import json

import tf_transformations as tft
import argparse

SKIP = 0

# Fixed camera model points and lines for frustum visualization.
CAM_POINTS = np.array(
    [
        [0.0, 0.0, 0.0],
        [-1.0, -1.0, 1.5],
        [1.0, -1.0, 1.5],
        [1.0, 1.0, 1.5],
        [-1.0, 1.0, 1.5],
        [-0.5, 1.0, 1.5],
        [0.5, 1.0, 1.5],
        [0.0, 1.2, 1.5],
    ],
    dtype=np.float32,
)

CAM_LINES = np.array(
    [[1, 2], [2, 3], [3, 4], [4, 1], [1, 0], [0, 2], [3, 0], [0, 4], [5, 7], [7, 6]],
    dtype=np.int32,
)

CAM_SCALE = 0.05


def transform_camera_points(translation, quaternion, scale=CAM_SCALE):
    """
    Transforms the fixed camera model points using the provided translation and quaternion.
    """
    q = [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
    T = tft.quaternion_matrix(q)
    T[0:3, 3] = translation
    pts = CAM_POINTS * scale
    ones = np.ones((pts.shape[0], 1), dtype=np.float32)
    pts_hom = np.hstack((pts, ones))
    transformed = (T @ pts_hom.T).T[:, :3]
    return transformed


def create_frustum_marker(pose, marker_id, is_latest=False):
    """
    Creates a Marker (LINE_LIST) representing the camera frustum for one pose.
    """
    translation, quaternion = pose
    pts = transform_camera_points(translation, quaternion)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = Clock().now().to_msg()
    marker.ns = "camera_frustums"
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.01

    if is_latest:
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    else:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    marker.color.a = 1.0

    for line in CAM_LINES:
        pt1 = Point()
        pt1.x, pt1.y, pt1.z = pts[line[0]].tolist()
        pt2 = Point()
        pt2.x, pt2.y, pt2.z = pts[line[1]].tolist()
        marker.points.append(pt1)
        marker.points.append(pt2)
    return marker


def create_point_cloud_msg(points, colors, frame_id="map"):
    msg = PointCloud2()
    msg.header.stamp = Clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = points.shape[0]
    msg.is_bigendian = False
    msg.is_dense = True
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16  # 12 bytes for xyz + 4 bytes for rgb
    msg.row_step = msg.point_step * points.shape[0]
    buffer = []
    for i in range(points.shape[0]):
        x, y, z = points[i]
        r, g, b = colors[i].astype(np.uint8)
        rgb_int = (r << 16) | (g << 8) | b
        rgb_float = struct.unpack("f", struct.pack("I", rgb_int))[0]
        buffer.append(struct.pack("ffff", x, y, z, rgb_float))
    msg.data = b"".join(buffer)
    return msg


class DPVOCombinedPublisher(Node):
    def __init__(self):
        super().__init__("dpvo_combined_publisher")
        self.br = TransformBroadcaster(self)
        self.trajectory_marker_pub = self.create_publisher(
            MarkerArray, "trajectory_markers", 10
        )
        self.frustum_marker_pub = self.create_publisher(
            MarkerArray, "camera_frustums", 10
        )
        self.point_cloud_pub = self.create_publisher(PointCloud2, "point_cloud", 10)
        self.image_pub = self.create_publisher(Image, "image_stream", 10)
        self.dpvo_reconstructed_path_pub = self.create_publisher(
            Path, "reconstructed_path", 10
        )

        self.create_timer(0.5, self.timer_callback)

        self.trajectory = None
        self.pc_points = None
        self.pc_colors = None
        self.image_data = None

        self.bridge = CvBridge()

    def update_trajectory(self, trajectory):
        self.trajectory = trajectory

    def update_point_cloud(self, points, colors):
        self.pc_points = points
        self.pc_colors = colors

    def update_image(self, image):
        self.image_data = image

    def timer_callback(self):
        # Publish current TF transform (from the last pose in trajectory).
        if self.trajectory is not None and self.trajectory.positions_xyz.shape[0] > 0:
            pos = self.trajectory.positions_xyz[-1]
            quat = self.trajectory.orientations_quat_wxyz[-1]
            tf_quat = [quat[1], quat[2], quat[3], quat[0]]
            t = TransformStamped()
            t.header.stamp = Clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "drone"
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation.x = float(tf_quat[0])
            t.transform.rotation.y = float(tf_quat[1])
            t.transform.rotation.z = float(tf_quat[2])
            t.transform.rotation.w = float(tf_quat[3])
            self.br.sendTransform(t)

        # Publish path
        if self.trajectory is not None and self.trajectory.positions_xyz.shape[0] > 0:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = Clock().now().to_msg()

            for i in range(self.trajectory.positions_xyz.shape[0]):
                pos = self.trajectory.positions_xyz[i]
                quat = self.trajectory.orientations_quat_wxyz[i]
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = Clock().now().to_msg()
                pose.pose.position.x = float(pos[0])
                pose.pose.position.y = float(pos[1])
                pose.pose.position.z = float(pos[2])
                pose.pose.orientation.x = float(quat[1])
                pose.pose.orientation.y = float(quat[2])
                pose.pose.orientation.z = float(quat[3])
                pose.pose.orientation.w = float(quat[0])
                path_msg.poses.append(pose)

            self.dpvo_reconstructed_path_pub.publish(path_msg)

        # Publish trajectory as a line strip.
        if self.trajectory is not None:
            traj_marker = Marker()
            traj_marker.header.frame_id = "map"
            traj_marker.header.stamp = Clock().now().to_msg()
            traj_marker.ns = "trajectory_path"
            traj_marker.id = 0
            traj_marker.type = Marker.LINE_STRIP
            traj_marker.action = Marker.ADD
            traj_marker.scale.x = 0.05
            traj_marker.color.a = 1.0
            traj_marker.color.r = 0.0
            traj_marker.color.g = 1.0
            traj_marker.color.b = 0.0
            for pos in self.trajectory.positions_xyz:
                pt = Point()
                pt.x = float(pos[0])
                pt.y = float(pos[1])
                pt.z = float(pos[2])
                traj_marker.points.append(pt)
            ma = MarkerArray()
            ma.markers.append(traj_marker)
            self.trajectory_marker_pub.publish(ma)

        # Publish camera frustum markers.
        if self.trajectory is not None:
            frustum_array = MarkerArray()
            num = self.trajectory.positions_xyz.shape[0]
            for i in range(num):
                pos = self.trajectory.positions_xyz[i]
                quat = self.trajectory.orientations_quat_wxyz[i]
                marker = create_frustum_marker(
                    (pos, quat), marker_id=i, is_latest=(i == num - 1)
                )
                frustum_array.markers.append(marker)
            self.frustum_marker_pub.publish(frustum_array)

        # Publish point cloud.
        if self.pc_points is not None and self.pc_colors is not None:
            pc_msg = create_point_cloud_msg(
                self.pc_points, self.pc_colors, frame_id="map"
            )
            self.point_cloud_pub.publish(pc_msg)

        # Publish image stream.
        if self.image_data is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(self.image_data, encoding="bgr8")
                img_msg.header.stamp = Clock().now().to_msg()
                img_msg.header.frame_id = "map"
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Image publishing failed: {e}")


@torch.no_grad()
def run_dpvo(
    cfg,
    network,
    imagedir,
    calib,
    stride=1,
    skip=0,
    viz=False,
    timeit=False,
    ros_pub_node=None,
):
    slam = None
    queue = Queue(maxsize=8)

    trajectory_positions = []
    trajectory_orientations = []
    trajectory_tstamps = []

    if os.path.isdir(imagedir):
        reader = Process(
            target=image_stream, args=(queue, imagedir, calib, stride, skip)
        )
    else:
        reader = Process(
            target=video_stream, args=(queue, imagedir, calib, stride, skip)
        )
    reader.start()

    while True:
        t, image, intrinsics = queue.get()
        if t < 0:
            break

        image = torch.from_numpy(image).permute(2, 0, 1).cuda()
        intrinsics = torch.from_numpy(intrinsics).cuda()

        if slam is None:
            _, H, W = image.shape
            slam = DPVO(cfg, network, ht=H, wd=W, viz=False)

        with Timer("SLAM", enabled=timeit):
            slam(t, image, intrinsics)

        if slam.n > 0:
            current_pose_array = (
                slam.pg.poses_[slam.n - 1].detach().cpu().numpy()
            )  # [x, y, z, qx, qy, qz, qw]
            translation = current_pose_array[:3]
            quat = np.array(
                [
                    current_pose_array[3],
                    current_pose_array[4],
                    current_pose_array[5],
                    current_pose_array[6],
                ]
            )

            def pose_to_matrix(translation, quat):
                T = tft.quaternion_matrix(quat)
                T[0:3, 3] = translation
                return T

            T = pose_to_matrix(translation, quat)
            T_inv = np.linalg.inv(T)
            center = T_inv[0:3, 3]

            # Convert from ENU to NED: (E, N, U) -> (N, E, -U)
            center_ned = np.array([center[1], center[0], -center[2]])

            trajectory_positions.append(center_ned)
            trajectory_orientations.append(quat)
            trajectory_tstamps.append(t)
        else:
            print(f"Warning: No valid pose returned at timestamp {t}")

        if trajectory_positions:
            current_traj = PoseTrajectory3D(
                positions_xyz=np.array(trajectory_positions),
                orientations_quat_wxyz=np.array(trajectory_orientations),
                timestamps=np.array(trajectory_tstamps),
            )
            if ros_pub_node is not None:
                ros_pub_node.update_trajectory(current_traj)
                pc_points = slam.pg.points_.cpu().numpy()[: slam.m]
                pc_colors = slam.pg.colors_.view(-1, 3).cpu().numpy()[: slam.m]
                ros_pub_node.update_point_cloud(pc_points, pc_colors)
                img_np = image.permute(1, 2, 0).cpu().numpy()
                ros_pub_node.update_image(img_np)

        torch.cuda.empty_cache()

    reader.join()
    points = slam.pg.points_.cpu().numpy()[: slam.m]
    colors = slam.pg.colors_.view(-1, 3).cpu().numpy()[: slam.m]
    return (
        slam.terminate(),
        (points, colors, (*intrinsics, H, W)),
        trajectory_positions,
        trajectory_orientations,
        trajectory_tstamps,
    )


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ("yes", "true", "t", "y", "1"):
        return True
    elif v.lower() in ("no", "false", "f", "n", "0"):
        return False
    else:
        raise argparse.ArgumentTypeError("Boolean value expected.")


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--network", type=str, default="dpvo.pth")
    parser.add_argument(
        "--imagedir",
        type=str,
        required=True,
        help="Path to image directory or video file",
    )
    parser.add_argument(
        "--calib", type=str, required=True, help="Path to calibration file"
    )
    parser.add_argument("--name", type=str, default="result", help="Name your run")
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--skip", type=int, default=0)
    parser.add_argument("--config", default="config/default.yaml")
    parser.add_argument("--timeit", action="store_true")
    parser.add_argument(
        "--viz",
        type=str2bool,
        nargs="?",
        const=True,
        default=False,
        help="Enable visualization (true/false)",
    )
    parser.add_argument(
        "--plot",
        type=str2bool,
        nargs="?",
        const=True,
        default=False,
        help="Enable ROS publishing (true/false)",
    )
    parser.add_argument("--opts", nargs="+", default=[])
    parser.add_argument("--save_ply", action="store_true")
    parser.add_argument("--save_colmap", action="store_true")
    parser.add_argument("--save_trajectory", action="store_true")
    args, _ = parser.parse_known_args()

    # Print current directory and arguments.
    print("Current directory: ", os.getcwd())
    print("Arguments: ", args)

    # Initialize the ROS context.
    rclpy.init(args=None)

    # --- Publish DPVO start parameters as a status message ---
    # Create a temporary node for status publishing.
    status_node = Node("dpvo_status_publisher")
    status_pub = status_node.create_publisher(String, "reconstructor_status", 10)

    status_msg = String()
    # Convert args to a dictionary and then to JSON.
    status_msg.data = json.dumps(vars(args))

    # Publish the status message.
    status_pub.publish(status_msg)
    status_node.get_logger().info("Published DPVO start parameters.")
    # Spin briefly to ensure the message is sent.
    rclpy.spin_once(status_node, timeout_sec=0.5)
    status_node.destroy_node()

    # Continue with your DPVO processing.
    if args.plot:
        ros_pub_node = DPVOCombinedPublisher()
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(ros_pub_node,), daemon=True
        )
        spin_thread.start()
    else:
        ros_pub_node = None

    # Merge configuration and run DPVO.
    cfg.merge_from_file(args.config)
    cfg.merge_from_list(args.opts)
    print("Running with config:")
    print(cfg)

    (
        result,
        extra,
        trajectory_positions,
        trajectory_orientations,
        trajectory_tstamps,
    ) = run_dpvo(
        cfg,
        args.network,
        args.imagedir,
        args.calib,
        args.stride,
        args.skip,
        args.viz,
        args.timeit,
        ros_pub_node=ros_pub_node,
    )

    # (The rest of your code to save trajectories, publish markers, etc., remains unchanged.)
    if args.plot and ros_pub_node is not None:
        ros_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
