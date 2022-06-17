import sys
from datetime import time
from functools import partial
from typing import Callable, List
from multiprocessing import cpu_count

import numpy as np
import rospy
import rospkg
import os
from multiprocessing import cpu_count, set_start_method

# from stable_baselines3 import PPO
# from supersuit.vector import MakeCPUAsyncConstructor, ConcatVecEnv
# from supersuit.vector.sb3_vector_wrapper import SB3VecEnvWrapper

from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.model.agent_factory import (
    AgentFactory,
)
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.model.base_agent import (
    BaseAgent,
)
from marl_tools.custom_mlp_utils import *
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.model.custom_policy import *
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.model.custom_sb3_policy import *

# from tools.argsparser import parse_training_args
from marl_tools.staged_train_callback import InitiateNewTrainStage

# remove, because we will read from yaml file
from marl_tools.argsparser import parse_marl_training_args
from tools.argsparser import parse_training_args

from arena_marl.marl_agent.envs.pettingzoo_env import env_fn

from arena_marl.marl_agent.utils.supersuit_utils import (
    vec_env_create,
)

# from tools.argsparser import parse_marl_training_args
from marl_tools.train_agent_utils import (
    get_MARL_agent_name_and_start_time,
    get_paths,
    choose_agent_model,
    load_config,
    initialize_hyperparameters,
)
from marl_tools import train_agent_utils
from typing import List

import os, sys, rospy, time

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import (
    SubprocVecEnv,
    DummyVecEnv,
    VecNormalize,
)
from stable_baselines3.common.callbacks import (
    EvalCallback,
    StopTrainingOnRewardThreshold,
    MarlEvalCallback,
)
from stable_baselines3.common.policies import BasePolicy

from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.train_agent_utils import *

from rl_agent.training_agent_wrapper import TrainingDRLAgent
from scripts.deployment.drl_agent_node import DeploymentDRLAgent

from nav_msgs.srv import GetMap


ROBOT_MODEL = rospy.get_param("model")
DEFAULT_HYPERPARAMETER = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    "hyperparameters",
    "default.json",
)
ROBOT_ACTION_SPACE = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    "action_spaces",
    f"default_settings_{ROBOT_MODEL}.yaml",
)


def instantiate_drl_agents(
    num_robots: int = 1,
    ns: str = None,
    robot_name_prefix: str = "robot",
    hyperparameter_path: str = DEFAULT_HYPERPARAMETER,
    action_space_path: str = ROBOT_ACTION_SPACE,
) -> List[TrainingDRLAgent]:
    """Function which generates a list agents which handle the ROS connection.

    Args:
        num_robots (int, optional): Number of robots in the environment. Defaults to 1.
        ns (str, optional): Name of the namespace (used for ROS topics). Defaults to None.
        robot_name_prefix (str, optional): Name with which to prefix robots in the ROS environment. Defaults to "robot".
        hyperparameter_path (str, optional): Path where to load hyperparameters from. Defaults to DEFAULT_HYPERPARAMETER.
        action_space_path (str, optional): Path where to load action spaces from. Defaults to DEFAULT_ACTION_SPACE.

    Returns:
        List[TrainingDRLAgent]: List containing _num\_robots_ agent classes.
    """
    return [
        TrainingDRLAgent(
            ns=ns,
            robot_name=robot_name_prefix + str(i + 1),
            hyperparameter_path=hyperparameter_path,
            action_space_path=action_space_path,
        )
        for i in range(num_robots)
    ]


def main(args):
    # load configuration
    config = load_config(args.config)

    robots = {}
    MARL_NAME, START_TIME = get_MARL_agent_name_and_start_time()

    # create seperate model instances for each robot
    for robot in config["robots"].items():
        # generate agent name and model specific paths
        robot_name, config_params = robot
        if config_params["resume"] is None:
            agent_name = robot_name + "_" + START_TIME
        else:
            agent_name = config_params["resume"].split("/")[-1]

        paths = train_agent_utils.get_paths(
            MARL_NAME,
            robot_name,
            agent_name,
            config_params,
            config["training_curriculum"],
            args,
        )
        # initialize hyperparameters (save to/ load from json)
        params = train_agent_utils.initialize_hyperparameters(
            PATHS=paths,
            config=config_params,
            n_envs=config["n_envs"],
        )

        env = vec_env_create(
            env_fn,
            instantiate_drl_agents,
            num_robots=config_params["num_robots"],
            num_cpus=cpu_count() - 1,
            num_vec_envs=config["n_envs"],
            PATHS=paths,
        )

        env = VecNormalize(
            env,
            training=True,
            norm_obs=True,
            norm_reward=True,
            clip_reward=15,
            clip_obs=15,
        )

        # TODO: add support for new config setup
        model = choose_agent_model(agent_name, paths, args, env, params)

        # add configuration for one robot to robots dictionary
        robots[robot_name] = {
            "model": model,
            "env": env,
            "n_envs": config["n_envs"],
            "config_params": config_params,
            "params": params,
            "paths": paths,
        }

    # set num of timesteps to be generated
    n_timesteps = 40000000 if args.n is None else args.n

    start = time.time()
    try:
        robots[robot_name][model].learn(
            total_timesteps=n_timesteps,
            reset_num_timesteps=True,
            # Übergib einfach das dict für den aktuellen roboter
            callback=get_evalcallback(
                robots[robot_name],
                # train_env=env,
                # num_robots=args.robots,
                # num_envs=args.n_envs,
                # task_mode=params["task_mode"],
                # PATHS=paths,
            ),
        )
    except KeyboardInterrupt:
        print("KeyboardInterrupt..")
    # finally:
    # update the timesteps the model has trained in total
    # update_total_timesteps_json(n_timesteps, PATHS)

    robots[robot_name][model].env.close()
    print(f"Time passed: {time.time() - start}s")
    print("Training script will be terminated")
    sys.exit()


def get_evalcallback(
    agent: dict,
    # train_env: VecEnv,
    # num_robots: int,
    # num_envs: int,
    # task_mode: str,
    # PATHS: dict,
) -> MarlEvalCallback:
    """Function which generates an evaluation callback with an evaluation environment.

    Args:
        train_env (VecEnv): Vectorized training environment
        num_robots (int): Number of robots in the environment
        num_envs (int): Number of parallel spawned environments
        task_mode (str): Task mode for the current experiment
        PATHS (dict): Dictionary which holds hyperparameters for the experiment

    Returns:
        MarlEvalCallback: [description]
    """
    eval_env = env_fn(
        num_agents=agent["config_params"]["num_robots"],
        ns="eval_sim",
        agent_list_fn=instantiate_drl_agents,
        max_num_moves_per_eps=700,
        PATHS=agent["paths"],
    )

    eval_env = VecNormalize(
        eval_env,
        training=False,
        norm_obs=True,
        norm_reward=False,
        clip_reward=15,
        clip_obs=3.5,
    )

    return MarlEvalCallback(
        train_env=agent["env"],
        eval_env=eval_env,
        num_robots=agent["config_params"]["num_robots"],
        n_eval_episodes=40,
        eval_freq=20000,
        deterministic=True,
        log_path=agent["paths"]["eval"],
        best_model_save_path=agent["paths"]["model"],
        callback_on_eval_end=InitiateNewTrainStage(
            n_envs=agent["n_envs"],
            treshhold_type="succ",
            upper_threshold=0.8,
            lower_threshold=0.6,
            task_mode=agent["params"]["task_mode"],
            verbose=1,
        ),
        callback_on_new_best=StopTrainingOnRewardThreshold(
            treshhold_type="succ",
            threshold=0.9,
            verbose=1,
        ),
    )


if __name__ == "__main__":
    set_start_method("fork")
    # args, _ = parse_marl_training_args()
    args, _ = parse_training_args()
    # rospy.init_node("train_env", disable_signals=False, anonymous=True)
    main(args)
