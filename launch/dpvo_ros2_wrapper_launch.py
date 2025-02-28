from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    dpvo_dir = os.path.expanduser("~/Documents/DPVO")

    if not os.path.exists(dpvo_dir):
        print(
            "DPVO directory does not exist. Please clone the DPVO repository to ~/Documents/DPVO"
        )
        exit(4)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imagedir",
                default_value="/home/gasper/Datasets/EuRoc/MH01/mav0/cam0/data",
                description="Path to the image directory or video file",
            ),
            DeclareLaunchArgument(
                "network",
                default_value=os.path.join(dpvo_dir, "dpvo.pth"),
                description="Path to the DPVO network file",
            ),
            DeclareLaunchArgument(
                "calib",
                default_value=os.path.join(dpvo_dir, "calib", "euroc.txt"),
                description="Path to the calibration file",
            ),
            DeclareLaunchArgument(
                "config",
                default_value=os.path.join(dpvo_dir, "config", "default.yaml"),
                description="Path to the DPVO configuration file",
            ),
            DeclareLaunchArgument(
                "stride", default_value="2", description="Stride parameter for DPVO"
            ),
            DeclareLaunchArgument(
                "skip", default_value="0", description="Skip parameter for DPVO"
            ),
            DeclareLaunchArgument(
                "viz", default_value="true", description="Enable visualization"
            ),
            DeclareLaunchArgument(
                "plot",
                default_value="true",
                description="Enable ROS TF, trajectory, point cloud, and image publishing",
            ),
            Node(
                package="dpvo_ros2_wrapper",
                executable="dpvo_ros2_publisher",
                name="dpvo_ros2_publisher",
                output="screen",
                exec_name="/home/gasper/micromamba/envs/dpvo/bin/python3",
                arguments=[
                    "--imagedir",
                    LaunchConfiguration("imagedir"),
                    "--calib",
                    LaunchConfiguration("calib"),
                    "--network",
                    LaunchConfiguration("network"),
                    "--config",
                    LaunchConfiguration("config"),
                    "--stride",
                    LaunchConfiguration("stride"),
                    "--skip",
                    LaunchConfiguration("skip"),
                    "--viz",
                    LaunchConfiguration("viz"),
                    "--plot",
                    LaunchConfiguration("plot"),
                ],
            ),
        ]
    )
