"""Start Gazebo Harmonic with a generated scenario world and the ROS-Gazebo bridge.
  ros2 launch cogndn_sim sim.launch.py scenario:=s1_livestock [map:=town|mcity_vla] [headless:=true]
map:=town  minimal synthetic town (default); map:=mcity_vla  earlier Mcity map (city_vla) under the same road graph."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('cogndn_sim')
    scen = LaunchConfiguration('scenario').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    wmap = LaunchConfiguration('map').perform(context)
    if wmap not in ('town', 'mcity_vla'):
        raise RuntimeError('map must be town or mcity_vla, got %r' % wmap)
    world = os.path.join(share, 'worlds', scen + ('_mcity_vla' if wmap == 'mcity_vla' else '') + '.sdf')
    bridge = 'bridge_mcity_vla.yaml' if wmap == 'mcity_vla' else 'bridge.yaml'
    gz_args = ('-r -s ' if headless else '-r ') + world
    return [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                                                       'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='cogndn_bridge', output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', bridge), 'use_sim_time': True}]),
    ] + ([Node(package='cogndn_sim', executable='street_bodies_node', name='street_bodies', output='screen',
               parameters=[{'scenario': scen, 'use_sim_time': True}])] if wmap == 'mcity_vla' else [])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='s0_straight_control'),
        DeclareLaunchArgument('map', default_value='mcity_vla'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=setup),
    ])
