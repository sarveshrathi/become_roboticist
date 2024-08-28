from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_param_builder import load_xacro
from os.path import join
from pathlib import Path

def generate_launch_description():
   path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
   #world=join(get_package_share_directory("maciv2"), "model","simple_gamecity_world.sdf")
   #gazebo_sim = IncludeLaunchDescription(path)
   cokeworld=join(get_package_share_directory("maciv2"),"model","coke_table.sdf")
   gazebo_sim = IncludeLaunchDescription(path, launch_arguments=[("gz_args",  cokeworld)])
   
   
   
   robot = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create", "-topic", "robot_description", '-z', '1.0'],
        name="spawn robot",
        output="both"
        )
   
   robot_file = join(get_package_share_directory("maciv2"), "robot_description","maciv2.urdf.xacro")
   robot_xml = load_xacro(Path(robot_file))
   
   robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description':robot_xml, 
                     'use_sim_time':True}],
    )
   
   #Controller manager package help to separate tasks like spawning, differential drive, changing the motor, etc. It is easier to focus on one small separated task than a small portion in big task

   start_controllers  = Node(
                package="controller_manager",
                executable="spawner",
                arguments=['joint_state_broadcaster', 'Arm_controller', 'Gripper_controller'],
                output="screen"
            )
   
   #Use next two launch files created from moveit function
   move_group = IncludeLaunchDescription(join(get_package_share_directory("maciv2_moveit"), "launch", "move_group.launch.py"))
   rviz = IncludeLaunchDescription(join(get_package_share_directory("maciv2_moveit"), "launch", "moveit_rviz.launch.py"))

   # Gazebo Bridge: This brings data (sensors/clock) out of gazebo into ROS.
   # Need to include this for simulation clock time
   bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
                   ],
        output='screen'        )
   
   #Add these additional parameters for configuring the simulation time
   mg_sim_time = ExecuteProcess(cmd=["ros2", "param", "set", "/move_group", "use_sim_time","True"])
   rviz_sim_time = ExecuteProcess(cmd=["ros2", "param", "set", "/rviz", "use_sim_time","True"])
   
   return LaunchDescription([gazebo_sim,robot,robot_state_publisher,start_controllers, move_group, rviz, bridge,mg_sim_time, rviz_sim_time])
#ros2 param set /move_group use_sim_time True