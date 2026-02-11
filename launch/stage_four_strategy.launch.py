from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import RegisterEventHandler, EmitEvent

from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare

from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import os
import lifecycle_msgs.msg

def generate_launch_description():
#Declare arguments
    declared_arguments = []
    uav_name = os.environ['UAV_NAME']
    declared_arguments.append(
        DeclareLaunchArgument(
            'stage_four_file',
            default_value=PathJoinSubstitution([FindPackageShare('stage_four_strategy'),
                                                'params', 'waypoint.yaml']),
            description='Full path to the file with the all parameters.'
        )
    )

#Initialize arguments
    manager_node_file = LaunchConfiguration('stage_four_file')

    manager_node_lifecycle_node = LifecycleNode(
        package='stage_four_strategy',
        executable='manager_node',
        name='manager_node',
        namespace='',
        output='screen',
        parameters=[manager_node_file],
        remappings=[
            ('/diagnostics_in', '/' + uav_name + '/control_manager/diagnostics'),
            ('/qrcode_detection_in', '/' + uav_name + '/qrcode/detections'),
            ('/trajectory_path_out', '/' + uav_name + '/control_manager/trajectory_path'), 
            ('/takeoff', '/' + uav_name + '/control_manager/takeoff'),
            ('/land', '/' + uav_name + '/control_manager/land'),
        ]
    )

    event_handlers = []

    event_handlers.append(
#Right after the node starts, make it take the 'configure' transition.
        RegisterEventHandler(
            OnProcessStart(
                target_action=manager_node_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(manager_node_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=manager_node_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(manager_node_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    ld = LaunchDescription()

#Declare the arguments
    for argument in declared_arguments:
        ld.add_action(argument)

#Add client node
    ld.add_action(manager_node_lifecycle_node)

#Add event handlers
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
