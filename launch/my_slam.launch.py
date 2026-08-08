from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'cloud_topic',
        default_value='/utlidar/cloud_deskewed',
        description='Bemeneti lidar pontfelho topic'
    ))

    ld.add_action(DeclareLaunchArgument(
        'map_save_path',
        default_value='/tmp/my_slam_map.pcd',
        description='Ide menti a terkepet a /my_slam/save_map szolgaltatas'
    ))

    slam_node = Node(
        package='my_slam',
        executable='my_slam',
        name='my_slam',
        output='screen',
        parameters=[{
            # --- topicok / frame-ek ---
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'fallback_sensor_frame': 'utlidar_lidar',
            'map_save_path': LaunchConfiguration('map_save_path'),

            # --- pontfelho szures (base_link frame-ben) ---
            'min_range': 0.4,
            'max_range': 40.0,
            'min_z_base': -1.0,
            'max_z_base': 3.0,

            # --- felbontasok ---
            'icp_leaf': 0.25,          # ICP-hez ritkitva (sebesseg)
            'map_leaf': 0.10,          # terkepbe kerulo felbontas
            'local_map_leaf': 0.20,    # lokalis terkep (illesztesi cel)
            'global_map_leaf': 0.10,   # globalis voxel terkep felbontasa

            # --- kulcskepek / lokalis terkep ---
            'keyframe_dist': 0.5,
            'keyframe_angle': 0.26,
            'local_map_radius': 40.0,
            'local_map_max_keyframes': 50,

            # --- ICP ---
            'max_iterations': 20,
            'min_correspondences': 60,
            'max_icp_points': 15000,
            'max_correspondence_dist': 1.0,
            'plane_threshold': 0.10,
            'huber_delta': 0.10,
            'max_position_correction': 1.0,
            'max_angle_correction': 0.35,

            # --- egyeb ---
            'tf_timeout': 0.05,
            'map_publish_period': 2.0,
            'num_threads': 4,
            'publish_registered_scan': True,
            'use_sim_time': False,
        }]
    )

    ld.add_action(slam_node)
    return ld