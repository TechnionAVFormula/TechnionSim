from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument        # noqa: F401
from launch.substitutions import LaunchConfiguration    # noqa: F401
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # track_name = DeclareLaunchArgument(
    #     "track_name",
    #     description="The name of the track"
    # )

    stephanos_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("stephanos_controller"),
                                       "/launch/stephanos_controller.launch.py"])
    )

    pure_pursuit_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("pure_pursuit_controller"),
                                       "/launch/pure_pursuit_controller.launch.py"])
    )

    # parameter_changer = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory("parameter_changer"),
    #                                    "/launch/parameter_changer.launch.py"])
    # )

    # eufs_joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory("eufs_joystick"),
    #                                    "/launch/ps4.py"])
    # )

    # lap_stats = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory("control_eval"),
    #                                    "/launch/lap_stats.launch.py"]),
    #     launch_arguments={
    #         "track_name": LaunchConfiguration("track_name")
    #     }.items()
    # )

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("planning_meta"),
                                       "/launch/planning.launch.py"])
    )

    ld = LaunchDescription()

    # NOTE: Comment/uncomment the lines below to launch different nodes
    # ld.add_action(stephanos_controller)
    ld.add_action(pure_pursuit_controller)
    # ld.add_action(eufs_joystick)

    # ld.add_action(parameter_changer)

    # ld.add_action(lap_stats); ld.add_action(track_name)

    # Uncomment this if you want to launch the planning nodes as well
    # NOTE: Make sure to not also uncomment launching control in the planning launch file
    ld.add_action(planning)

    return ld
