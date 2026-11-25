import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros


def generate_launch_description():
    planner_type_arg = DeclareLaunchArgument(
        "planner_type",
        default_value=os.environ.get("PNC_PLANNER_TYPE", "em"),
        description="Planner algorithm type (em, lattice)",
    )
    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value=os.environ.get("PNC_CONTROLLER_TYPE", "mpc"),
        description="Controller algorithm type (mpc, lqr, stanley)",
    )

    return launch.LaunchDescription([
        planner_type_arg,
        controller_type_arg,
        launch_ros.actions.Node(
            package="my_pnc",
            executable="planning_agent",
            parameters=[{"planner_type": LaunchConfiguration("planner_type")}],
        ),
        launch_ros.actions.Node(
            package="my_pnc",
            executable="control_agent",
            parameters=[{"controller_type": LaunchConfiguration("controller_type")}],
        ),
        launch_ros.actions.Node(
            package="my_pnc",
            executable="planning_debug_plotter.py",
            name="planning_debug_plotter",
        ),
    ])
