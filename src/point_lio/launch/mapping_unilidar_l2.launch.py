# <launch>
# 	<!-- Launch file for Velodyne16 VLP-16 LiDAR -->

# 	<arg name="rviz" default="true" />

# 	<node pkg="point_lio_unilidar" type="pointlio_mapping" name="laserMapping" output="screen">
# 		<rosparam command="load" file="$(find point_lio_unilidar)/config/unilidar_l2.yaml" />
		
# 		<!--change to 1 to use IMU as input of Point-LIO-->
# 		<param name="use_imu_as_input" type="bool" value="0"/>
		
# 		<param name="prop_at_freq_of_imu" type="bool" value="1"/>
# 		<param name="check_satu" type="bool" value="1"/>
# 		<param name="init_map_size" type="int" value="10"/>
# 		<param name="point_filter_num" type="int" value="1"/> <!--4, 3-->

# 		<param name="space_down_sample" type="bool" value="1" />
# 		<param name="filter_size_surf" type="double" value="0.1" /> <!--0.5, 0.3, 0.2, 0.15, 0.1-->
# 		<param name="filter_size_map" type="double" value="0.1" /> <!--0.5, 0.3, 0.15, 0.1-->
# 		<param name="cube_side_length" type="double" value="1000" /> <!--2000-->
# 		<param name="runtime_pos_log_enable" type="bool" value="0" /> <!--1-->
# 	</node>

# 	<group if="$(arg rviz)">
# 		<node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz" args="-d $(find point_lio_unilidar)/rviz_cfg/loam_livox.rviz" />
# 	</group> launch-prefix="gdb -ex run --args"

# </launch>
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')

    # Node parameters, including those from the YAML configuration file
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'unilidar_l2.yaml'
        ]),
        {
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,  # Options: 4, 3
            'space_down_sample': True,
            'filter_size_surf': 0.1,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.1,  # Options: 0.5, 0.3, 0.15, 0.1
            'cube_side_length': 1000.0,  # Option: 1000 (changed from 2000)
            'runtime_pos_log_enable': False,  # Option: True
        }
    ]

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        # prefix='gdb -ex run --args'
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # Assemble the launch description
    ld = LaunchDescription([
        rviz_arg,
        laser_mapping_node,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld
