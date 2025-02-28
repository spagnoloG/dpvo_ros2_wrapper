from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def find_python_environment_path(env_name):
    home = os.path.expanduser("~")
    possible_paths = [
        os.path.join(home, "micromamba", "envs", env_name, "bin", "python3"),
        os.path.join(home, "miniconda3", "envs", env_name, "bin", "python3"),
        os.path.join(home, "anaconda3", "envs", env_name, "bin", "python3"),
    ]
    for path in possible_paths:
        if os.path.exists(path):
            return path
    return None


def generate_launch_description():
    dpvo_dir = os.path.expanduser("~/Documents/DPVO")
    pkg_share = get_package_share_directory("dpvo_ros2_wrapper")
    if not os.path.exists(dpvo_dir):
        print(
            "DPVO directory does not exist. Please clone the DPVO repository to ~/Documents/DPVO"
        )
        exit(4)

    python_executable = find_python_environment_path("dpvo")
    if python_executable is None:
        print("Python environment 'dpvo' not found.")
        exit(5)

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
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=["-d", os.path.join(pkg_share, "config", "dpvo.rviz")],
            ),
            Node(
                package="dpvo_ros2_wrapper",
                executable="dpvo_ros2_publisher",
                name="dpvo_ros2_publisher",
                output="screen",
                exec_name=python_executable,
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
