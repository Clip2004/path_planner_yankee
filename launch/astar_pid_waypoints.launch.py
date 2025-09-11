# library to move between files and folders in the O.S.
import os
from ament_index_python.packages import get_package_share_directory
# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Ruta al archivo YAML dentro del paquete f112th_sim_2502_yankee
    config_path = os.path.join(
        get_package_share_directory('f112th_sim_2502_yankee'),
        'config',
        'max_expand_params.yaml'   # asegúrate de que este sea el nombre del archivo
    )

    path_planner_Astar_node_yankee = Node(
        package='path_planner_yankee',
        executable='path_planner_Astar_node_yankee',
        name='path_planner_Astar_node_yankee',
        parameters=[config_path]  # aquí cargamos el archivo de parámetros
    )

    path_tracker_pid_waypoints_node_yankee = Node(
        package='path_tracker_yankee',
        executable='path_tracker_pid_waypoints_node_yankee',
        name='path_tracker_pid_waypoints_node_yankee',
    )

    goal_manager_node = Node(
        package='path_planner_yankee',
        executable='goal_manager_node_yankee',
        name='goal_manager_node_yankee',
    )

    # Launch them all!
    return LaunchDescription([
        path_planner_Astar_node_yankee,
        path_tracker_pid_waypoints_node_yankee,
        goal_manager_node
    ])
