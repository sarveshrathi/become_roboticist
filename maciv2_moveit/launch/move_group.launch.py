from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("maciv2", package_name="maciv2_moveit").to_moveit_configs()
    return generate_move_group_launch(moveit_config)

'''    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [move_group_node]
    )
'''