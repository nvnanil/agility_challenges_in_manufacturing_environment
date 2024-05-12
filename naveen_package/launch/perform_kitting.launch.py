import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, AllSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    # Determine which tutorial to run
    # tutorial_num = int(LaunchConfiguration("tutorial").perform(context))

    # if not 1 <= tutorial_num <= 8:
    #     print("tutorial argument must be a number between 1 and 8")
    #     exit()

    # Create parameters dictionary
    parameters = {"use_sim_time": True}

    # Update parameters dictionary with moveit items if necessary
    parameters["use_moveit"] = True

    urdf = os.path.join(get_package_share_directory("ariac_description"), "urdf/ariac_robots/ariac_robots.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("ariac_robots", package_name="ariac_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/ariac_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("ariac_tutorials")
            + "/config/moveit_config.yaml"
        )
        .to_moveit_configs()
    )

    parameters.update(moveit_config.to_dict())

    # Tutorial Node
    kitting_node = Node(
        package="naveen_package",
        executable=f"order_kitting.py",
        output="screen",
        parameters=[
            parameters
        ]
    )

    # Move Group node
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_moveit_config"), "/launch", "/ariac_robots_moveit.launch.py"]
        ),
        condition=IfCondition(str(parameters["use_moveit"]))
    )

    nodes_to_start = [
        kitting_node,
        move_group,
    ]

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])