#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from koi.koi import KoiStudy
from koi.utils import add_cylinder_link_to_urdf, add_fake_link_to_urdf, add_fixed_joint_to_urdf, \
    add_material_to_urdf, add_revolute_joint_to_urdf, create_standard_parser, \
    get_parameter_dict_from_trial
from optuna import Trial
import yaml


RADIUS = 0.1
REVOLUTE_LIMIT = math.pi


class SerialRobot(KoiStudy):
    """A simple serial robot with cylindrical links."""

    def __init__(self, optimize_base: bool = False, min_number_joints=5, max_number_joints=6,
                 min_link_length=0.05, max_link_length=1.0, *args, **kwargs) -> None:
        """
        Initialize the serial robot.

        Args:
            optimize_base (bool, optional): Whether to optimize the base pos. Defaults to False.
            min_number_joints (int, optional): The minimum number of joints. Defaults to 5.
            max_number_joints (int, optional): The maximum number of joints. Defaults to 6.
            min_link_length (float, optional): The minimum link length. Defaults to 0.05.
            max_link_length (float, optional): The maximum link length. Defaults to 1.0.
        """
        self.optimize_base = optimize_base
        self.min_number_joints = min_number_joints
        self.max_number_joints = max_number_joints
        self.min_link_length = min_link_length
        self.max_link_length = max_link_length

        self.koi_dir = get_package_share_directory('koi')
        config_path = f'{self.koi_dir}/demo/config/demo.yaml'
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        if self.optimize_base:
            config['display']['kinematic_base_frame'] = 'moved_base_link'
        super().__init__(config,
                         f'{self.koi_dir}/demo/models/kinematics.yaml',
                         f'{self.koi_dir}/demo/models/joint_limits.yaml', *args, **kwargs)

    def suggest_parameters(self, trial: Trial) -> None:
        """
        Suggest parameters for the serial robot.

        These are the number of joints, their axis and the length of each link.
        Optionally, this can also be the position of the base.

        Args:
            trial (Trial): The current trial.
        Returns:
            parameters (dict): The suggested parameters.
        """
        parameters = {}

        if self.optimize_base:
            parameters['base_position_x'] = trial.suggest_float('base_position_x', -1.0, 1.0)
            parameters['base_position_y'] = trial.suggest_float('base_position_y', -1.0, 1.0)
            parameters['base_position_z'] = trial.suggest_float('base_position_z', -1.0, 1.0)
        else:
            trial.set_user_attr('base_position_x', 0.0)
            trial.set_user_attr('base_position_y', 0.0)
            trial.set_user_attr('base_position_z', 0.0)

        # only suggest this parameter if it is not fixed
        if self.min_number_joints == self.max_number_joints:
            parameters['number_joints'] = self.min_number_joints
            trial.set_user_attr('number_joints', parameters['number_joints'])
        parameters['number_joints'] = trial.suggest_int(
            'number_joints', self.min_number_joints, self.max_number_joints)

        for i in range(parameters['number_joints'] + 1):
            parameters[f'link_length_{i}'] = trial.suggest_float(
                f'link_length_{i}', self.min_link_length, self.max_link_length)
            parameters[f'joint_axis_{i}'] = trial.suggest_categorical(
                f'joint_axis_{i}', ['0 0 1', '0 1 0', '1 0 0'])
        return parameters

    def additional_objective_values(self, trial: Trial) -> list[float]:
        """
        Return additional objective values for the given trial.

        These are the number of joints and the summed up link length.
        Args:
            trial (Trial): Optuna trial object.
        Returns:
            objective_values (list[float]): Additional objective values.
        """
        # give objective values based on number of joints and summed up link length
        values = []
        parameters = get_parameter_dict_from_trial(trial)
        values.append(float(parameters['number_joints']))
        values.append(sum(parameters[f'link_length_{i}']
                          for i in range(parameters['number_joints'] + 1)))
        return values

    def get_directions(self) -> list[str]:
        """
        Return the optimization directions for the study.

        Returns:
            directions (list[str]): Optimization directions.
        """
        return ['maximize', 'minimize', 'minimize']

    def get_objective_names(self) -> list[str]:
        """
        Return the objective names for the study.

        Returns:
            objective_names (list[str]): Objective names.
        """
        if self.use_reach_score:
            return ['Reach Score', 'No. Joints', 'Sum. Link Length']
        else:
            return ['Reach Percentage', 'No. Joints', 'Sum. Link Length']

    def create_moveit_model(self, parameters) -> tuple[str, str]:
        """
        Create a urdf and srdf based on the suggested parameter values.

        Args:
            parameters (dict): Suggested parameter values.
        Returns:
            urdf (str): URDF as a string.
            srdf (str): SRDF as a string.
        """
        # We use a base urdf which specifies the UR arm and the endoscope box.
        # The part that is optimized is then generated based on the parameters
        with open(f'{self.koi_dir}/demo/models/base.urdf', 'r') as urdf_file:
            urdf_string = urdf_file.read()
        urdf = ET.fromstring(urdf_string)
        add_material_to_urdf(urdf, 'red', '1 0 0 1')
        add_material_to_urdf(urdf, 'blue', '0 0 1 1')
        add_cylinder_link_to_urdf(
            urdf, 'link_0', parameters['link_length_0'], RADIUS, 'blue')
        add_fixed_joint_to_urdf(
            urdf, 'fixed_base', 'base_link', 'link_0', pitch=-math.pi/2)
        for i in range(parameters['number_joints']):
            # alternating colors to make it easier to see
            if i % 2 == 0:
                color = 'blue'
            else:
                color = 'red'
            add_cylinder_link_to_urdf(
                urdf, f'link_{i+1}', parameters[f'link_length_{i+1}'], RADIUS, color)
            add_revolute_joint_to_urdf(
                urdf, f'joint_{i}', f'link_{i}', f'link_{i+1}', -REVOLUTE_LIMIT, REVOLUTE_LIMIT,
                x=parameters[f'link_length_{i}'], axis_str=parameters[f'joint_axis_{i}'])
        add_fake_link_to_urdf(urdf, 'end_effector')
        # endeffector link with offset so that it is possible to get to target without collisons
        add_fixed_joint_to_urdf(
            urdf, 'fixed_end', f"link_{parameters['number_joints']}", 'end_effector',
            x=parameters[f"link_length_{parameters['number_joints']}"]+0.1, roll=math.pi,
            pitch=-math.pi/2)
        urdf = ET.tostring(urdf, encoding='utf8', method='xml').decode('utf-8')

        with open(f'{self.koi_dir}/demo/models/robot.srdf', 'r') as srdf_file:
            srdf_string = srdf_file.read()
        srdf = ET.fromstring(srdf_string)
        # make sure that links that are adjacent to each other do not collide
        for i in range(parameters['number_joints']):
            disable_collision_element = ET.Element('disable_collisions')
            disable_collision_element.set('link1', f'link_{i}')
            disable_collision_element.set('link2', f'link_{i+1}')
            disable_collision_element.set('reason', 'Adjacent')
            srdf.append(disable_collision_element)
        srdf = ET.tostring(srdf, encoding='utf8', method='xml').decode('utf-8')
        return urdf, srdf


if __name__ == '__main__':
    parser = create_standard_parser()
    parser.add_argument('--optimize-base', action='store_true')
    parser.add_argument('--min-number-joints', help='Minimal number of joints to be tried out',
                        default=5, type=int, required=False)
    parser.add_argument('--max-number-joints', help='Maximal number of joints to be tried out',
                        default=6, type=int, required=False)
    parser.add_argument('--min-link-length', help='Minimal link length to be tried out',
                        default=0.05, type=float, required=False)
    parser.add_argument('--max-link-length', help='Maximal link length to be tried out',
                        default=1.0, type=float, required=False)
    args = parser.parse_args()

    koi = SerialRobot(**vars(args))
    koi.run(**vars(args))
