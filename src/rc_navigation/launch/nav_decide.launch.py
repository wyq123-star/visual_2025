#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile  # 关键修改：导入参数文件加载器

def generate_launch_description():
    config_file_path = PathJoinSubstitution([
        FindPackageShare('rc_navigation'),
        'config',  
        'nav_decide_real.yaml'
    ])
    
    config_path_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_path,
        description='全局参数配置文件路径'
    )
    
    # 使用ParameterFile加载YAML内容
    param_file = ParameterFile(
        LaunchConfiguration('config_file'),
        allow_substs=True
    )

    nodes = [
        Node(
            package='my_algorithm',
            executable='random_generate.py',
            name='random_generate',
            parameters=[param_file]  # 传递参数内容而非路径
        ),
        Node(
            package='change_laser',
            executable='obstacle_extractor_node',
            name='obstacle_extractor',
            parameters=[param_file]  # 所有节点共用同一参数文件
        ),
        Node(
            package='my_algorithm',
            executable='logic.py',
            name='optimal_point_selector',
            parameters=[param_file]
        ),
        Node(
            package='my_algorithm',
            executable='nav_behave.py',
            emulate_tty=True,
            output='screen',
            name='optimal_goal_navigator',
            parameters=[param_file]
        )
    ]
    
    return LaunchDescription([
        config_path_arg,
        LogInfo(msg="启动导航决策..."),
        *nodes,
        LogInfo(msg=f"参数文件路径: {config_file_path}") 
    ])