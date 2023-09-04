import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    yolox_ros_share_dir = get_package_share_directory('yolox_ros_py')

    # video_device = LaunchConfiguration('video_device', default='/dev/video0')
    # video_device_arg = DeclareLaunchArgument(
    #     'video_device',
    #     default_value='/dev/video0',
    #     description='Video device'
    # )

    # webcam = launch_ros.actions.Node(
    #     package="v4l2_camera", executable="v4l2_camera_node",
    #     parameters=[
    #         {"image_size": [640,480]},
    #         {"video_device": video_device},
    #     ],
    # )

    dog_path = os.path.join(yolox_ros_share_dir, "./", "dog.jpg")
    url = "https://raw.githubusercontent.com/pjreddie/darknet/master/data/dog.jpg"
    if not os.path.exists(dog_path):
        os.system("wget {} -O {}".format(url, dog_path))


    image_path = LaunchConfiguration('image_path', default=dog_path)
    image_path_arg = DeclareLaunchArgument(
        'image_path',
        default_value=dog_path,
        description='Image path'
    )

    image_pub = launch_ros.actions.Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='image_publisher',
        arguments=[image_path],
    )

    yolox_ros = launch_ros.actions.Node(
        package="yolox_ros_py", executable="yolox_ros",
        parameters=[
            {"yolox_exp_py" : yolox_ros_share_dir+'/yolox_nano.py'},
            {"device" : 'cpu'},
            {"fp16" : True},
            {"fuse" : False},
            {"legacy" : False},
            {"trt" : False},
            {"ckpt" : yolox_ros_share_dir+"/yolox_nano.pth"},
            {"conf" : 0.3},
            {"threshold" : 0.65},
            {"resize" : 640},
        ],
    )

    rqt_graph = launch_ros.actions.Node(
        package="rqt_graph", executable="rqt_graph",
    )

    return launch.LaunchDescription([
        # video_device_arg,
        # webcam,
        image_path_arg,
        image_pub,
        yolox_ros,
        # rqt_graph
    ])