import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='hybrid_planning_container',
            namespace='/',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='moveit_hybrid_planning',
                    plugin='moveit::hybrid_planning::GlobalPlannerComponent',
                    name='global_planner'),
                ComposableNode(
                    package='moveit_hybrid_planning',
                    plugin='moveit::hybrid_planning::LocalPlannerComponent',
                    name='local_planner'),
                ComposableNode(
                    package='moveit_hybrid_planning',
                    plugin='moveit::hybrid_planning::HybridPlanningManager',
                    name='hybrid_planning_manager')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])