import argparse
import xml.etree.ElementTree as ET

from rclpy.parameter import Parameter, PARAMETER_SEPARATOR_STRING
from rclpy.serialization import serialize_message
import yaml


def add_material_to_urdf(urdf, name, color):
    """
    Add a material to the given URDF.

    Args:
        urdf (xml.etree.ElementTree.Element): URDF as an ElementTree.
        name (str): Name of the material.
        color (str): Color of the material.
    """
    material = ET.Element('material')
    material.set('name', name)
    color_element = ET.Element('color')
    color_element.set('rgba', color)
    material.append(color_element)
    urdf.append(material)


def add_cylinder_link_to_urdf(urdf, name, length, radius, material=None):
    """
    Add a cylinder link to the given URDF.

    Args:
        urdf (xml.etree.ElementTree.Element): URDF as an ElementTree.
        name (str): Name of the link.
        length (float): Length of the cylinder.
        radius (float): Radius of the cylinder.
    """
    link = ET.Element('link')
    link.set('name', name)
    visual = ET.Element('visual')
    geometry = ET.Element('geometry')
    cylinder = ET.Element('cylinder')
    cylinder.set('radius', str(radius))
    cylinder.set('length', str(length))
    geometry.append(cylinder)
    origin = ET.Element('origin')
    origin.set('xyz', f'{length/2} 0 0')
    origin.set('rpy', '0 1.570796327 0')
    if material is not None:
        # first do a sanity check to make sure that the material exists
        if urdf.find(f'material[@name="{material}"]') is None:
            raise ValueError(
                f'Material {material} does not exist. Use the add_material_to_urdf() function to add it.')
        material_element = ET.Element('material')
        material_element.set('name', material)
        visual.append(material_element)
    visual.append(geometry)
    visual.append(origin)
    collision = ET.Element('collision')
    collision.append(geometry)
    collision.append(origin)
    link.append(visual)
    link.append(collision)
    urdf.append(link)


def add_fake_link_to_urdf(urdf, name):
    """
    Add a fake link to the given URDF.

    Args:
        urdf (xml.etree.ElementTree.Element): URDF as an ElementTree.
        name (str): Name of the link.
    """
    link = ET.Element('link')
    link.set('name', name)
    urdf.append(link)


def add_fixed_joint_to_urdf(urdf, name, parent_name, child_name, x=0.0, y=0.0, z=0.0, roll=0.0,
                            pitch=0.0, yaw=0.0):
    """
    Add a fixed joint to the given URDF.

    Args:
        urdf (xml.etree.ElementTree.Element): URDF as an ElementTree.
        name (str): Name of the joint.
        parent_name (str): Name of the parent link.
        child_name (str): Name of the child link.
        x (float): X coordinate of the origin.
        y (float): Y coordinate of the origin.
        z (float): Z coordinate of the origin.
        roll (float): Roll of the origin.
        pitch (float): Pitch of the origin.
        yaw (float): Yaw of the origin.
    """
    joint = ET.Element('joint')
    joint.set('name', name)
    joint.set('type', 'fixed')
    parent = ET.Element('parent')
    parent.set('link', parent_name)
    joint.append(parent)
    child = ET.Element('child')
    child.set('link', child_name)
    joint.append(child)
    origin = ET.Element('origin')
    origin.set('xyz', f'{x} {y} {z}')
    origin.set('rpy', f'{roll} {pitch} {yaw}')
    joint.append(origin)
    urdf.append(joint)


def add_revolute_joint_to_urdf(urdf, name, parent_name, child_name, lower_limit, upper_limit, x=0.0,
                               y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, axis_str='0 1 0'):
    """
    Add a revolute joint to the given URDF.

    Args:
        urdf (xml.etree.ElementTree.Element): URDF as an ElementTree.
        name (str): Name of the joint.
        parent_name (str): Name of the parent link.
        child_name (str): Name of the child link.
        lower_limit (float): Lower limit of the joint.
        upper_limit (float): Upper limit of the joint.
        x (float): X coordinate of the origin.
        y (float): Y coordinate of the origin.
        z (float): Z coordinate of the origin.
        roll (float): Roll of the origin.
        pitch (float): Pitch of the origin.
        yaw (float): Yaw of the origin.
        axis_str (str): Axis of the joint.
    """
    joint = ET.Element('joint')
    joint.set('name', name)
    joint.set('type', 'revolute')
    origin = ET.Element('origin')
    origin.set('xyz', f'{x} {y} {z}')
    origin.set('rpy', f'{roll} {pitch} {yaw}')
    joint.append(origin)
    parent = ET.Element('parent')
    parent.set('link', parent_name)
    joint.append(parent)
    child = ET.Element('child')
    child.set('link', child_name)
    joint.append(child)
    axis = ET.Element('axis')
    axis.set('xyz', axis_str)
    joint.append(axis)
    limits = ET.Element('limit')
    limits.set('lower', str(lower_limit))
    limits.set('upper', str(upper_limit))
    limits.set('effort', '5.0')
    limits.set('velocity', '0.5')
    joint.append(limits)
    urdf.append(joint)


def add_2dof_joint_to_urdf(urdf, name1, name2, parent_name, child_name, lower_limit1, lower_limit2,
                           upper_limit1, upper_limit2, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0,
                           yaw=0.0):
    """
    Add a 2dof joint to the given URDF.

    Args:
        urdf (xml.etree.ElementTree.Element): URDF as an ElementTree.
        name1 (str): Name of the first joint.
        name2 (str): Name of the second joint.
        parent_name (str): Name of the parent link.
        child_name (str): Name of the child link.
        lower_limit1 (float): Lower limit of the first joint.
        lower_limit2 (float): Lower limit of the second joint.
        upper_limit1 (float): Upper limit of the first joint.
        upper_limit2 (float): Upper limit of the second joint.
        x (float): X coordinate of the origin.
        y (float): Y coordinate of the origin.
        z (float): Z coordinate of the origin.
        roll (float): Roll of the origin.
        pitch (float): Pitch of the origin.
        yaw (float): Yaw of the origin.
    """
    fake_link_name = f'fake_link_{name1}_{name2}'
    add_fake_link_to_urdf(urdf, fake_link_name)
    add_revolute_joint_to_urdf(urdf, name1, parent_name, fake_link_name,
                               lower_limit1, upper_limit1, x, y, z, roll, pitch, yaw)
    add_revolute_joint_to_urdf(urdf, name2, fake_link_name, child_name,
                               lower_limit2, upper_limit2, axis_str='0 0 1')


def serialize_parameters(parameters):
    """
    Serialize a list of parameters.

    Args:
        parameters (list): List of parameters.

    Returns:
        list: List of serialized parameters.        
    """
    serialized_parameters = []
    for parameter in parameters:
        serialized_parameters.append(serialize_message(parameter))
        if parameter.value.type == 2 and 'solver_attempts' not in parameter.name:
            print(f'Gave parameter {parameter.name} of integer type. If the code crashes it is '
                  f'maybe because this should be a float. You may need to add an .0 in some yaml '
                  f'file.')
    return serialized_parameters


def load_moveit_parameters(urdf: str, srdf: str, kinematics_yaml_path: str,
                           joint_limits_yaml_path: str):
    """Load the moveit parameters from files and return them.

    This method is based on\
        https://github.com/bit-bots/bitbots_misc/blob/master/bitbots_utils/bitbots_utils/utils.py.
    Args:
        urdf (str): Path to the urdf file.
        srdf (str): Path to the srdf file.
        kinematics_yaml_path (str): Path to the kinematics yaml file.
        joint_limits_yaml_path (str): Path to the joint limits yaml file.
    Returns:
        moveit_parameters (list): List of moveit parameters.
    """
    moveit_parameters = []
    kinematics_parameters = get_parameters_from_plain_yaml(
        f'{kinematics_yaml_path}',
        'robot_description_kinematics.')
    moveit_parameters = moveit_parameters + kinematics_parameters
    joint_limit_parameters = get_parameters_from_plain_yaml(
        f'{joint_limits_yaml_path}',
        'robot_description_joint_limits.')
    moveit_parameters = moveit_parameters + joint_limit_parameters
    moveit_parameters.append(Parameter(name='robot_description', value=urdf))
    moveit_parameters.append(
        Parameter(name='robot_description_semantic', value=srdf))
    return moveit_parameters


def get_parameters_from_plain_yaml(parameter_file, namespace=''):
    """
    Get parameters from a yaml file.

    Method from \
        https://github.com/bit-bots/bitbots_misc/blob/master/bitbots_utils/bitbots_utils/utils.py
    Args:
        parameter_file (str): Path to the yaml file.
        namespace (str): Namespace of the parameters.
    Returns:
        moveit_parameters (list): List of moveit parameters.
    """
    with open(parameter_file, 'r') as f:
        param_dict = yaml.safe_load(f)
        return parse_parameter_dict(namespace=namespace, parameter_dict=param_dict)


def parse_parameter_dict(*, namespace, parameter_dict):
    """
    Parse a dictionary of parameters into a list of parameters by unrolling nested parameters.

    Based on \
        https://github.com/bit-bots/bitbots_misc/blob/master/bitbots_utils/bitbots_utils/utils.py
    Args:
        namespace (str): Namespace of the parameters.
        parameter_dict (dict): Dictionary of parameters.
    Returns:
        moveit_parameters (list): List of moveit parameters.
    """
    parameters = []
    for param_name, param_value in parameter_dict.items():
        full_param_name = namespace + param_name
        # Unroll nested parameters
        if isinstance(param_value, dict):
            parameters += parse_parameter_dict(
                namespace=full_param_name + PARAMETER_SEPARATOR_STRING,
                parameter_dict=param_value)
        else:
            parameters.append(
                Parameter(name=full_param_name, value=param_value))
    return parameters


def get_parameter_dict_from_trial(trial):
    """
    Get a dictionary of parameters from a trial.

    Args:
        trial (optuna.trial.Trial): Trial from which the parameters should be extracted.

    Returns:
        parameters (dict): Dictionary of parameters.
    """
    parameters = {}
    for parameter_name in trial.params.keys():
        parameters[parameter_name] = trial.params[parameter_name]
    # also set fixed parameters from user attributes
    for parameter_name in trial.user_attrs.keys():
        parameters[parameter_name] = trial.user_attrs[parameter_name]
    return parameters


def create_standard_parser():
    """
    Create a standard parser with the arguments necessary for any KOI study.

    Returns:
        parser (argparse.ArgumentParser): Standard parser.    
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--journal-file', help='Path to where the journal file for this study is \
                        located', type=str, required=True)
    parser.add_argument('--name', help='Name of the study', type=str, required=True)
    parser.add_argument('--trials', help='Trials to be evaluated', default=10000, type=int,
                        required=False)
    parser.add_argument('--sampler', help='Sampler to be used', type=str, required=False,
                        default='tpe', choices=['tpe', 'random', 'nsgaii', 'cmaes', 'nsgaiii'])
    parser.add_argument('-m', '--multi-objective', help='Enables multi-objective optimization',
                        default=False, required=False, action='store_true')
    parser.add_argument('-n', '--no-opt', help='Disables the optimization steps of REACH',
                        default=False, required=False, action='store_true')
    parser.add_argument('-v', '--viz', help='Enables visualization', default=False, required=False,
                        action='store_true')
    parser.add_argument('-i', '--interactive', help='Interactive mode that waits for user after \
                        each trial', default=False, required=False, action='store_true')
    parser.add_argument('-b', '--show-best-trial', help='Load the best trial and show it',
                        default=False, required=False, action='store_true')
    parser.add_argument('--show-trial-number', help='Load this specific trial number and show it',
                        default=None, type=int, required=False)
    parser.add_argument('--use-reach-score', help='Use percentage of the reach space as evaluation \
                        metric', default=False, action='store_true', required=False)
    parser.add_argument('--enable-ros-logging', help='Enables ROS logging', default=False,
                        action='store_true', required=False)
    return parser
