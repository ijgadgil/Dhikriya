"""Full Cognitive-NDN stack: simulator + perception + VLA + compiler + NDN consumer + driver +
safety envelope + explanations.

  ros2 launch cogndn_bringup cogndn.launch.py scenario:=s3_closure_detour policy:=cognitive model:=llava-phi3 map:=mcity_vla
policy: cognitive | cognitive_hybrid | route_typed | route_all | ondemand
        ablations: cognitive_vlm (no camera OCR) | cognitive_ocr (OCR sign head only, no vision-language model)
The NDN data plane (forwarders + producer) and Redis must already be running (run_scenario.sh does this).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(ctx):
    a = {k: LaunchConfiguration(k).perform(ctx) for k in ('scenario', 'policy', 'model', 'headless', 'map', 'cruise', 'k',
                                                           'use_context', 'yolo_model')}
    cognitive = a['policy'].startswith('cognitive')
    params = os.path.join(get_package_share_directory('cogndn_bringup'), 'config', 'params.yaml')
    common = [params, {'scenario': a['scenario']}]
    nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cogndn_sim'), 'launch', 'sim.launch.py')),
            launch_arguments={'scenario': a['scenario'], 'headless': a['headless'], 'map': a['map']}.items()),
        Node(package='cogndn_perception', executable='yolo_node', output='screen',
             parameters=[params, {'model': a['yolo_model']}]),
        Node(package='cogndn_perception', executable='lidar_cluster_node', output='screen', parameters=[params]),
        Node(package='cogndn_compiler', executable='compiler_node', output='screen',
             parameters=common + [{'policy': a['policy'], 'k': int(a['k'])}]),
        Node(package='cogndn_ndn', executable='consumer_node', output='screen', parameters=[params]),
        Node(package='cogndn_navigation', executable='driver_node', output='screen',
             parameters=common + [{'cruise': float(a['cruise']), 'follow_vla': cognitive}]),
        Node(package='cogndn_navigation', executable='safety_envelope', output='screen', parameters=[params]),
    ]
    if cognitive:
        nodes += [
            Node(package='cogndn_cognition', executable='vla_node', output='screen',
                 parameters=common + [{'model': a['model'], 'use_context': a['use_context'] == 'true',
                                       'ocr': a['policy'] != 'cognitive_vlm',
                                       'use_model': a['policy'] != 'cognitive_ocr'}]),
            Node(package='cogndn_explain', executable='explain_node', output='screen', parameters=[params]),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='s1_livestock'),
        DeclareLaunchArgument('policy', default_value='cognitive'),
        DeclareLaunchArgument('model', default_value='llava-phi3'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('map', default_value='mcity_vla'),
        DeclareLaunchArgument('cruise', default_value='3.0'),
        DeclareLaunchArgument('k', default_value='3'),
        DeclareLaunchArgument('use_context', default_value='true'),
        DeclareLaunchArgument('yolo_model', default_value='yolov10n.pt'),
        OpaqueFunction(function=setup),
    ])
