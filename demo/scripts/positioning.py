#!/usr/bin/env python3

from math import pi
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from koi.koi import KoiStudy
from koi.utils import add_fake_link_to_urdf, add_fixed_joint_to_urdf, create_standard_parser
from optuna import Trial
import yaml


class PositioningDemo(KoiStudy):
    """This class is used to optimize the position of the panda robot in the default REACH demo."""

    def __init__(self, initial_guess, orientation, *args, **kwargs) -> None:
        """
        Initilize the demo study.

        Since it is a simple demo, there is not much to do here.
        """
        self.orientation = orientation
        self.reach_ros_dir = get_package_share_directory('reach_ros')
        with open(f'{self.reach_ros_dir}/demo/config/reach_study.yaml', 'r') as f:
            config = yaml.safe_load(f)
        config['display']['kinematic_base_frame'] = 'moved_base_link'
        super().__init__(config, f'{self.reach_ros_dir}/demo/model/kinematics.yaml',
                         f'{self.reach_ros_dir}/demo/model/joint_limits.yaml', *args, **kwargs)
        if initial_guess:
            guess = {
                'base_position_x': 0.0,
                'base_position_y': 0.0,
                'base_position_z': 0.0
            }
            if orientation:
                guess['base_roll'] = 0.0
                guess['base_pitch'] = 0.0
            self.study.enqueue_trial(guess)

    def suggest_parameters(self, trial: Trial) -> dict:
        """
        Suggest parameters for the study.

        Here only the position of the base is optimized.

        Args:
            trial (Trial): Optuna trial.
        Returns:
            parameters (dict): Suggested parameters.
        """
        parameters = {}
        parameters['base_position_x'] = trial.suggest_float('base_position_x', -1.0, 0.25)
        parameters['base_position_y'] = trial.suggest_float('base_position_y', -1.0, 1.0)
        parameters['base_position_z'] = trial.suggest_float('base_position_z', -1.0, 1.0)
        if self.orientation:
            parameters['base_roll'] = trial.suggest_float('base_roll', -pi, pi)
            parameters['base_pitch'] = trial.suggest_float('base_pitch', -pi, pi)
            parameters['base_yaw'] = trial.suggest_float('base_yaw', -pi, pi)
        return parameters

    def create_moveit_model(self, parameters) -> tuple[str, str]:
        """
        Create a urdf and srdf based on the suggested parameter values.

        We mostly use the default URDF and SRDF from the reach_ros package.
        But add a new root link to the URDF to be able to move the robot's base.

        Args:
            parameters (dict): Suggested parameter values.
        Returns:
            urdf (str): URDF as a string.
            srdf (str): SRDF as a string.
        """
        # since the file is xacro, we first need to compile it to a normal URDF
        urdf_string = subprocess.run(
            ['xacro', f'{self.reach_ros_dir}/demo/model/reach_study.xacro'],
            stdout=subprocess.PIPE).stdout.decode('utf-8')
        # as we need to move the robot's base for each optimization interation, we add a new root
        # link to the URDF this link will then be used by MoveIt to transform all IK goals since it
        # is the root link of the robot
        urdf = ET.fromstring(urdf_string)
        add_fake_link_to_urdf(urdf, 'moved_base_link')
        add_fixed_joint_to_urdf(urdf, 'moved_base_joint', 'moved_base_link', 'base_link',
                                parameters['base_position_x'], parameters['base_position_y'],
                                parameters['base_position_z'], parameters['base_roll'], parameters['base_pitch'], parameters['base_yaw'])
        urdf_string = ET.tostring(urdf, encoding='utf8', method='xml').decode('utf-8')
        # we don't want to change anything in the srdf, just read the files and return them
        with open(f'{self.reach_ros_dir}/demo/model/reach_study.srdf', 'r') as srdf_file:
            srdf = srdf_file.read()

        return urdf_string, srdf


if __name__ == '__main__':
    parser = create_standard_parser()
    parser.add_argument('--initial-guess', action='store_true',
                        help='Use an initial guess of 0 pose.')
    parser.add_argument('--orientation', '-o', action='store_true',
                        help='Optimize the orientation of the base.')
    args, unknown = parser.parse_known_args()

    koi = PositioningDemo(**vars(args))
    koi.run(**vars(args))
