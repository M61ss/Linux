import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('adder-correct_launch'),
      'config',
      'adder-correct_config.yaml'
      )

   return LaunchDescription([
      Node(
         package='adder-correct',
         executable='sum_node',
         name='sum_node',
         parameters=[config]
      )
   ])