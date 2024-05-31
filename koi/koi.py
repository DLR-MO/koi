import os

from koi.utils import get_parameter_dict_from_trial, load_moveit_parameters
import optuna
from optuna import Trial
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
import reach
import reach_ros


class KoiStudy:
    """
    Koi is the main class of the KOI framework.

    Any KOI optimizer should inherit from this class.
    It defines the main optimization loop.
    Some methods need to be implemented in subclasses as these need to be adapted for the special
    use case.
    """

    def __init__(self, config, kinematics_yaml_path, joint_limits_yaml_path, name=None,
                 journal_file=None, sampler=None, multi_objective=False, no_opt=False, viz=False,
                 interactive=False, use_reach_score=False, enable_ros_logging=False, *args,
                 **kwargs) -> None:
        """
        Initialize the KOI framework.

        Args:
            config (dict): The configuration dictionary.
            kinematics_yaml_path (str): The path to the kinematics yaml file.
            joint_limits_yaml_path (str): The path to the joint limits yaml file.
            name (str): The name of the study.
            journal_file (str): The path to the journal file.
            sampler (optuna.samplers.BaseSampler): The sampler to use.
            multi_objective (bool): Whether to use multi-objective optimization.
            no_opt (bool): Whether to run without optimization.
            viz (bool): Whether to use visualization.
            interactive (bool): Whether to use interactive mode.
            use_reach_score (bool): Whether to use the reach score.
            enable_ros_logging (bool): Whether to enable ROS logging.
        """
        if name is None:
            print('No study name provided. This is not supported.')
            exit()
        if journal_file is None:
            print('No journal file provided. This is not supported.')
            exit()

        self.config = config
        self.kinematics_yaml_path = kinematics_yaml_path
        self.joint_limits_yaml_path = joint_limits_yaml_path
        self.study_name = name
        self.result_path = journal_file
        self.multi_objective = multi_objective
        self.no_opt = no_opt
        self.visualization = viz
        self.interactive = interactive
        self.use_reach_score = use_reach_score

        self.parameters = {}
        if self.multi_objective:
            self.directions = self.get_directions()
            self.objective_names = self.get_objective_names()
        else:
            self.directions = ['maximize']
            if self.use_reach_score:
                self.objective_names = ['Reach Score']
            else:
                self.objective_names = ['Reach Percentage']

        # Initialize ROS in C++ and Python with disabling logging spam as default
        ros_args = []
        if not enable_ros_logging:
            ros_args = ['--ros-args', '--disable-stdout-logs']
        reach_ros.init_ros(ros_args)
        if self.visualization:
            # We also need a Python ROS node to publish additional debug data (mostly the updated
            # robot description)
            rclpy.init(args=ros_args)
            self.node = rclpy.create_node('koi_study')
            self.cli = self.node.create_client(
                SetParameters, '/robot_state_publisher/set_parameters')
            while not self.cli.wait_for_service(timeout_sec=10.0):
                print('Could not connect to robot state publisher service')
            self.req = SetParameters.Request()
        else:
            # remove any display plugin to reduce CPU load
            self.config['display']['name'] = 'NoOpDisplay'
        if self.no_opt:
            # set optimization steps in the config to 0
            self.config['optimization']['max_steps'] = 0
        else:
            # one reach optimization step is enough
            self.config['optimization']['max_steps'] = 1

        sampler_object = None
        if sampler == 'tpe':
            sampler_object = optuna.samplers.TPESampler(
                constant_liar=True, multivariate=True, warn_independent_sampling=False)
        elif sampler == 'random':
            sampler_object = optuna.samplers.RandomSampler()
        elif sampler == 'nsgaii':
            sampler_object = optuna.samplers.NSGAIISampler()
        elif sampler == 'nsgaiii':
            sampler_object = optuna.samplers.NSGAIIISampler()
        elif sampler == 'cmaes':
            sampler_object = optuna.samplers.CmaesSampler()
        else:
            raise ValueError(f'Sampler {sampler} not supported')
        storage = optuna.storages.JournalStorage(
            optuna.storages.JournalFileStorage(journal_file))
        if len(self.directions) == 1:
            self.study = optuna.create_study(study_name=name, storage=storage,
                                             direction=self.directions[0], sampler=sampler_object,
                                             load_if_exists=True)
        else:
            self.study = optuna.create_study(study_name=name, storage=storage,
                                             directions=self.directions, sampler=sampler_object,
                                             load_if_exists=True)
        # save the used arguments as additional data in the study
        for kwarg in kwargs.keys():
            self.study.set_user_attr(kwarg, kwargs[kwarg])
        # save the name of the child class as a user attribute
        self.study.set_user_attr('script', type(self).__name__)

    def run(self, trials=100, show_trial_number=None, show_best_trial=False, *args, **kwargs) \
            -> None:
        """
        Run the optimization loop.

        Args:
            trials (int): The number of trials to run.
            show_trial_number (int): The number of the trial to show.
            show_best_trial (bool): Whether to show the best trial.
        """
        # See if we want to load a trial or run a new study
        trial = None
        if show_trial_number is not None:
            trial = self.study.get_trials()[show_trial_number]
        elif show_best_trial:
            if len(self.directions) == 1:
                trial = self.study.best_trial
            else:
                trials = self.study.best_trials
                print('Best trials:')
                text = '  Number'
                for objective_name in self.objective_names:
                    text += f' | {objective_name}'
                print(text)
                for trial in trials:
                    text = f'  {trial.number}'
                    for i in range(self.objective_names):
                        text += f' | \t{round(trial.values[i], 2)}'
                    print(text)
                trial_string = input('Chose a trial to show:')
                if trial_string == '':
                    exit()
                trial_number = int(trial_string)
                print(f'Showing trial {trial_number}')
                trial = self.study.trials[trial_number]
        if trial is not None:
            print(
                f'Loading trial {trial.number} with the following parameters:')
            for key, value in trial.params.items():
                print(f'  {key}: {value}')
            urdf, srdf = self.create_moveit_model(get_parameter_dict_from_trial(trial))
            self.run_reach_study(trial.user_attrs['result_path'], urdf, srdf)
        else:
            print('Running a new study.')
            self.study.optimize(self.objective, n_trials=trials)

    def suggest_parameters(self, trial: Trial) -> dict:
        """
        Suggest the parameters for the given trial.

        Args:
            trial (Trial): Optuna trial object.
        Returns:
            parameters (dict): Suggested parameters.
        """
        raise NotImplementedError()

    def create_moveit_model(self, parameters: dict) -> tuple[str, str]:
        """
        Create a urdf and srdf based on the suggested parameter values.

        Needs to be implemented in subclasses.
        Args:
            parameters (dict): Suggested parameter values.
        Returns:
            urdf (str): URDF as a string.
            srdf (str): SRDF as a string.
        """
        raise NotImplementedError()

    def additional_objective_values(self, trial: Trial) -> list[float]:
        """
        Return additional objective values for the given trial.

        Needs to be implemented in subclasses.
        Args:
            trial (Trial): Optuna trial object.
        Returns:
            objective_values (list[float]): Additional objective values.
        """
        return []

    def get_directions(self) -> list[str]:
        """
        Return the optimization directions for the study.

        Needs to be implemented in subclasses for multi-objective optimization.
        Returns:
            directions (list[str]): Optimization directions.
        """
        raise NotImplementedError()

    def get_objective_names(self) -> list[str]:
        """
        Return the objective names for the study.

        Needs to be implemented in subclasses for multi-objective optimization.
        Returns:
            objective_names (list[str]): Objective names.
        """
        raise NotImplementedError()

    def update_robot_state_publisher(self, urdf: str) -> None:
        """
        Update the robot state publisher with the given URDF by setting its parameter.

        Args:
            urdf (str): URDF as a string.
        """
        val = ParameterValue(string_value=urdf,
                             type=ParameterType.PARAMETER_STRING)
        self.req.parameters = [Parameter(name='robot_description', value=val)]
        self.future = self.cli.call_async(self.req)
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                return

    def run_reach_study(self, result_path: str, urdf: str, srdf: str) -> bool:
        """
        Run a Reach study with the given parameters.

        Args:
            result_path (str): Path to the results folder.
            urdf (str): URDF as a string.
            srdf (str): SRDF as a string.
        Returns:
            success (bool): True if the study was successful.
        """
        # load the MoveIt parameters and transfer them to c++
        parameters = load_moveit_parameters(
            urdf, srdf, self.kinematics_yaml_path, self.joint_limits_yaml_path)
        for parameter in parameters:
            reach_ros.set_parameter(parameter.name, parameter.value)
        # Update model for visualization
        if self.visualization:
            self.update_robot_state_publisher(urdf)

        reach.runReachStudy(self.config, self.study_name,
                            result_path, self.interactive)
        return True

    def get_score_of_result(self, result_path, trial):
        """
        Get the score of the given result path.

        Args:
            result_path (str): Path to the results folder.
        Returns:
            score (float): The score of the given result path.
        """
        reach_study_file = f'{result_path}/{self.study_name}/reach.db.xml'
        try:
            db = reach.load(reach_study_file)
        except:
            print(f'Could not find a reach study file at {reach_study_file}')
            exit(1)
        results = db.calculateResults()
        trial.set_user_attr('reach_percentage', results.reach_percentage)
        trial.set_user_attr('total_pose_score', results.total_pose_score)
        trial.set_user_attr('norm_total_pose_score', results.norm_total_pose_score)
        if self.use_reach_score:
            return results.total_pose_score
        else:
            return results.reach_percentage

    def objective(self, trial: Trial) -> float:
        """
        Optuna objective function which will be given to the study.

        Args:
            trial (Trial): Optuna trial object.
        Returns:
            float: Objective value.
        """
        parameters = self.suggest_parameters(trial)
        urdf, srdf = self.create_moveit_model(parameters)
        # create folders
        base_folder = f'{self.result_path}_{trial.study.study_name}'
        result_path = f'{self.result_path}_{trial.study.study_name}/{trial.number}'
        os.makedirs(base_folder, exist_ok=True)
        os.makedirs(result_path, exist_ok=True)
        # run the reach study
        trial.set_user_attr('result_path', result_path)
        success = self.run_reach_study(result_path, urdf, srdf)
        values = []
        if success:
            values.append(self.get_score_of_result(result_path, trial))
        else:
            # return NaN which will lead to Optuna treating the trial as fail
            values.append(float('nan'))
        if self.multi_objective:
            values = values + self.additional_objective_values(trial)
        # if we have just one objective value we can return it directly to allow
        # single-objective optimization
        if len(values) == 1:
            return values[0]
        else:
            return values
