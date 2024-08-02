#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    simple_amr_sim_path = get_package_share_directory("simple_amr_sim")
    world_file = LaunchConfiguration("world_file", default = join(simple_amr_sim_path, "worlds", "empty.sdf"))

    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    lidar_height = LaunchConfiguration("lidar_height")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_description': Command( \
                    ['xacro ', join(simple_amr_sim_path, 'urdf/robot.xacro'),
                    ' camera_enabled:=', camera_enabled,
                    ' lidar_height:=', lidar_height,
                    ' sim_gz:=', "true"
                    ])}],
        remappings=[
            ('/joint_states', 'simple_amr_sim/joint_states'),
        ]
    )



    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "simple_amr_sim",
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        "/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        "/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        "/kinect_camera@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
        "stereo_camera/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
        "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        "/kinect_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        "/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
        "/world/default/model/simple_amr_sim/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
        "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        ],
        remappings=[
        ('/world/default/model/simple_amr_sim/joint_state', 'simple_amr_sim/joint_states'),
        ('/ground_truth', 'simple_amr_sim/odom'),
        ('/scan/points', 'simple_amr_sim/scan'),
        ('/kinect_camera', 'simple_amr_sim/stereo_camera/right/image_raw'),  
        ('/stereo_camera/left/image_raw', 'simple_amr_sim/stereo_camera/left/image_raw'),
        ('/stereo_camera/right/image_raw', 'simple_amr_sim/stereo_camera/right/image_raw'),
        ('/imu', 'simple_amr_sim/imu'),
        ('/navsat', 'simple_amr_sim/navsat'),
        ('kinect_camera/camera_info', 'simple_amr_sim/kinect_camera/camera_info'),
        ('stereo_camera/left/camera_info', 'simple_amr_sim/stereo_camera/left/camera_info'),
        ('stereo_camera/right/camera_info', 'simple_amr_sim/stereo_camera/right/camera_info'),
        ('/kinect_camera/points', 'simple_amr_sim/kinect_camera/points'),
        ]
    )


# NOTE: A static transform publisher is required here to establish a fixed transform between the "kinect_camera" frame and the "base_link" frame of the robot. This transform is necessary for correctly positioning and orienting the Kinect camera with respect to the robot's base.

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "kinect_camera",
                    "--child-frame-id", "simple_amr_sim/base_link/kinect_camera"]
    )
    

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("lidar_height", default_value="0.1"),
        robot_state_publisher,
        gz_sim, gz_spawn_entity, gz_ros2_bridge,
        transform_publisher
    ])

