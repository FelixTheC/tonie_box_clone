from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():  # this name is required in exactly this name
    ld = LaunchDescription()

    pause_btn_node = Node(
        package="cpp_pkg",
        executable="pause_button"
    )

    volume_up_btn_node = Node(
        package="cpp_pkg",
        executable="volume_up_button"
    )

    volume_down_btn_node = Node(
        package="cpp_pkg",
        executable="volume_down_button"
    )

    mfrc522_reader_node = Node(
        package="cpp_pkg",
        executable="mfrc522_reader"
    )

    database_server_node = Node(
        package="python_pkg",
        executable="database_server"
    )

    audio_loop_node = Node(
        package="python_pkg",
        executable="audio_loop"
    )

    ld.add_action(pause_btn_node)
    ld.add_action(volume_up_btn_node)
    ld.add_action(volume_down_btn_node)
    ld.add_action(mfrc522_reader_node)
    ld.add_action(database_server_node)
    ld.add_action(audio_loop_node)

    return ld
