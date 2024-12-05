import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('sum_launch'),
      'config',
      'sum_config.yaml'
      )

   return LaunchDescription([
      Node(
         package='sum',
         executable='sum_node',
         namespace='sum_node',
         name='node',
         parameters=[config]
      )
   ])