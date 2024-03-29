""" Hyperparameters for PR2 policy optimization experiment. """
from __future__ import division

from datetime import datetime
import os.path

import numpy as np

from gps import __file__ as gps_filepath
from gps.agent.ros.agent_ros import AgentROS
from gps.algorithm.algorithm_badmm import AlgorithmBADMM
from gps.algorithm.cost.cost_fk import CostFK
from gps.algorithm.cost.cost_action import CostAction
from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.cost.cost_utils import RAMP_LINEAR, RAMP_FINAL_ONLY
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.policy_opt.outsource_policy_opt_to_ros import OutsourcePolicyOptToROS
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.policy.lin_gauss_init import init_lqr
from gps.algorithm.policy.policy_prior_gmm import PolicyPriorGMM
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE, RGB_IMAGE
from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info
import os

IMAGE_WIDTH = 240
IMAGE_HEIGHT = 240
IMAGE_CHANNELS = 3


EE_POINTS = np.array([[0.02, -0.025, 0.05], [0.02, -0.025, -0.05],
                      [0.02, 0.05, 0.0]])

SENSOR_DIMS = {
    JOINT_ANGLES: 7,
    JOINT_VELOCITIES: 7,
    END_EFFECTOR_POINTS: 3 * EE_POINTS.shape[0],
    END_EFFECTOR_POINT_VELOCITIES: 3 * EE_POINTS.shape[0],
    ACTION: 7,
    RGB_IMAGE: IMAGE_WIDTH*IMAGE_HEIGHT*IMAGE_CHANNELS,
}

PR2_GAINS = np.array([3.09, 1.08, 0.393, 0.674, 0.111, 0.152, 0.098])

EXP_DIR = os.path.dirname(os.path.realpath(__file__))

common = {
    'experiment_name': 'my_experiment' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'data_files_dir': os.path.join(EXP_DIR, 'data_files/'),
    'log_filename': os.path.join(EXP_DIR, 'log.txt'),
    'conditions': 2,
}

x0s = []
ee_tgts = []
reset_conditions = []
ee_pos_tgts = []

# Set up each condition.
for i in range(common['conditions']):
    ja_x0 = [0.7672968905628448, -0.44148370735571607, 0.002002266876363734, 0.7476533775853085, 0.0041948141609911005, 1.254699270858885, -0.0028972254115142704]

    ee_pos_x0 = np.array([[0.8496260063310132, -0.27029780966027417, -0.016246145055006794]])
    ee_rot_x0 = np.array([[-0.9998277972567736, 0.015613921575623742, 0.010029022170598383], [0.015670869143830205, 0.999861381940975, 0.005625012298718447], [-0.009939803466117067, 0.005781207150245945, -0.9999338867899922]])

    ja_aux = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]

    ee_pos_tgt = np.array([[0.8596260063310132, -0.12029780966027417+i*0.2, -0.116246145055006794]])
    ee_rot_tgt = np.array([[-0.9183327592979894, 0.27621409730918317, 0.2834972938985746], [0.297429466960146, 0.9541441924491688, 0.03383152670700853], [-0.26115255193841225, 0.11538904828100763, -0.9583760807495452]])

    x0 = np.zeros(32)
    x0[:7] = ja_x0
    x0[14:(14+9)] = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_x0, ee_rot_x0).T
    )

    ee_tgt = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T
    )

    aux_x0 = np.zeros(7)
    aux_x0[:] = ja_aux

    reset_condition = {
        TRIAL_ARM: {
            'mode': JOINT_SPACE,
            'data': x0[0:7],
        },
        AUXILIARY_ARM: {
            'mode': JOINT_SPACE,
            'data': aux_x0,
        },
    }

    x0s.append(x0)
    ee_tgts.append(ee_tgt)
    reset_conditions.append(reset_condition)
    ee_pos_tgts.append(ee_pos_tgt)

if not os.path.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

agent = {
    'type': AgentROS,
    'dt': 0.05,
    'conditions': common['conditions'],
    'T': 100,
    'x0': x0s,
    'ee_points_tgt': ee_tgts,
    'reset_conditions': reset_conditions,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                    END_EFFECTOR_POINT_VELOCITIES, RGB_IMAGE],
}

algorithm = {
    'type': AlgorithmBADMM,
    'conditions': common['conditions'],
    'iterations': 20,
    'lg_step_schedule': np.array([1e-4, 1e-3, 1e-2, 1e-1]),
    'policy_dual_rate': 0.1,
    'ent_reg_schedule': np.array([1e-3, 1e-3, 1e-2, 1e-1]),
    'fixed_lg_step': 3,
    'kl_step': 5.0,
    'init_pol_wt': 0.01,
    'min_step_mult': 0.01,
    'max_step_mult': 1.0,
    'sample_decrease_var': 0.05,
    'sample_increase_var': 0.1,
    'exp_step_increase': 2.0,
    'exp_step_decrease': 0.5,
    'exp_step_upper': 0.5,
    'exp_step_lower': 1.0,
    'max_policy_samples': 6,
    'policy_sample_mode': 'add',
}

algorithm['init_traj_distr'] = {
    'type': init_lqr,
    'init_gains':  1.0 / PR2_GAINS,
    'init_acc': np.zeros(SENSOR_DIMS[ACTION]),
    'init_var': 1.0,
    'stiffness': 0.5,
    'stiffness_vel': 0.25,
    'final_weight': 50,
    'dt': agent['dt'],
    'T': agent['T'],
}

torque_cost = {
    'type': CostAction,
    'wu': 5e-3 / PR2_GAINS,
}

fk_cost1 = {
    'type': CostFK,
    # Target end effector is subtracted out of EE_POINTS in ROS so goal
    # is 0.
    'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'l1': 0.1,
    'l2': 0.0001,
    'ramp_option': RAMP_LINEAR,
}

fk_cost2 = {
    'type': CostFK,
    'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'l1': 1.0,
    'l2': 0.0,
    'wp_final_multiplier': 10.0,  # Weight multiplier on final timestep.
    'ramp_option': RAMP_FINAL_ONLY,
}

algorithm['cost'] = {
    'type': CostSum,
    'costs': [torque_cost, fk_cost1, fk_cost2],
    'weights': [1.0, 1.0, 1.0],
}

algorithm['dynamics'] = {
    'type': DynamicsLRPrior,
    'regularization': 1e-6,
    'prior': {
        'type': DynamicsPriorGMM,
        'max_clusters': 20,
        'min_samples_per_cluster': 40,
        'max_samples': 20,
    },
}

algorithm['traj_opt'] = {
    'type': TrajOptLQRPython,
}

algorithm['policy_opt'] = {
    'type': OutsourcePolicyOptToROS,
    'weights_file_prefix': os.path.join(EXP_DIR, 'policy'),
    'iterations': 3000,
}

algorithm['policy_prior'] = {
    'type': PolicyPriorGMM,
    'max_clusters': 20,
    'min_samples_per_cluster': 40,
    'max_samples': 40,
}

import random
from datetime import datetime
config = {
    'iterations': algorithm['iterations'],
    'common': common,
    'verbose_trials': 0,
    'verbose_policy_trials': 1,
    'agent': agent,
    'gui_on': False,
    'algorithm': algorithm,
    'num_samples': 5,
    'use_gpu':True,
    'gpu_id': 0,
}

common['info'] = generate_experiment_info(config)
