import os
import yaml
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# para_dir = os.path.join(get_package_share_directory('phoxi_camera'), 'config', 'defaultparams.yaml')##yaml file contains parameters
# para_dir = '/home/zyadan/ros2_ws_camera/src/phoxi_camera/config/defaultparams.yaml';##yaml file contains parameters

configurable_parameters = [{'name': 'start_acquisition_',            'default': 'false', 'description': 'camera start acquisition'},
                           {'name': 'stop_acquisition_',             'default': 'true', 'description': 'camera start acquisition'},
                           {'name': 'log_level',                     'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'config_file',                   'default': "''", 'description': 'yaml config file'},

                           {'name': 'scanner_id_',                   'default': '', 'description': 'scanner id '},
                           {'name': 'frame_id',                      'default': 'PhoXi3Dscanner_sensor', 'description': 'frame id'},
                           {'name': 'latch_topics_',                 'default': 'false', 'description': ''},
                           {'name': 'topic_queue_size_',             'default': '1', 'description': ''},
                           {'name': 'init_from_config_',             'default': 'false', 'description': ''},
                           {'name': 'organized_cloud_',              'default': 'false', 'description': ''},
                           {'name': 'resolution_',                   'default': '1', 'description': ''},
                           {'name': 'scan_multiplier_',              'default': '1', 'description': ''},
                           {'name': 'confidence_',                   'default': '3.0', 'description': ''},
                           {'name': 'send_confidence_map_',          'default': 'true', 'description': ''},
                           {'name': 'send_depth_map_',               'default': 'true', 'description': ''},
                           {'name': 'send_normal_map_',              'default': 'true', 'description': ''},
                           {'name': 'send_point_cloud_',             'default': 'true', 'description': ''},
                           {'name': 'send_texture_',                 'default': 'true', 'description': ''},
                           {'name': 'shutter_multiplier_',           'default': '1', 'description': ''},
                           {'name': 'timeout_',                      'default': '-3', 'description': ''},
                           {'name': 'trigger_mode_',                 'default': '1', 'description': ''},
                           {'name': 'coordinate_space_',             'default': '1', 'description': ''},
                           {'name': 'ambient_light_suppression_',    'default': 'false', 'description': ''},
                           {'name': 'single_pattern_exposure_',      'default': '2', 'description': ''},
                           {'name': 'camera_only_mode_',             'default': 'false', 'description': ''},

                          ]     




def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
    

def launch_setup(context, *args, **kwargs):
    _config_file = LaunchConfiguration("config_file").perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)
    log_level = 'info'

    return [
        launch_ros.actions.Node(
            package='phoxi_camera',
            # namespace='phoxi_camera',
            name="phoxi_camera_node",
            executable='phoxi_camera_node',
            parameters=[set_configurable_parameters(configurable_parameters)
                        , params_from_file
                        ],
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
            )
    ]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=launch_setup)
    ])




if __name__ == '__main__':
    generate_launch_description()



