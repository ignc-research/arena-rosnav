import sys
import os, sys, rospy, time

from datetime import time
from multiprocessing import cpu_count

import rospy
from multiprocessing import cpu_count, set_start_method

# from stable_baselines3 import PPO
# from supersuit.vector import MakeCPUAsyncConstructor, ConcatVecEnv
# from supersuit.vector.sb3_vector_wrapper import SB3VecEnvWrapper

# from marl_tools.custom_mlp_utils import *
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.model.custom_policy import *
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.model.custom_sb3_policy import *

# from tools.argsparser import parse_training_args
from marl_tools.staged_train_callback import InitiateNewTrainStage

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

from stable_baselines3.common.callbacks import (
    StopTrainingOnRewardThreshold,
    MarlEvalCallback,
)

from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.train_agent_utils import *

from marl_agent.utils.utils import instantiate_train_drl_agents


def main(args):
    # load configuration
    config = load_config(args.config)

    # set debug_mode
    rospy.set_param("debug_mode", config["debug_mode"])

    robots, existing_robots = {}, 0
    MARL_NAME, START_TIME = get_MARL_agent_name_and_start_time()

    # create seperate model instances for each robot
    for robot_name, robot_train_params in config["robots"].items():
        # generate agent name and model specific paths
        agent_name = (
            robot_train_params["resume"].split("/")[-1]
            if robot_train_params["resume"]
            else f"{robot_name}_{START_TIME}"
        )

        paths = train_agent_utils.get_paths(
            MARL_NAME,
            robot_name,
            agent_name,
            robot_train_params,
            config["training_curriculum"]["training_curriculum_file"],
            config["eval_log"],
            config["tb"],
        )

        # initialize hyperparameters (save to/ load from json)
        hyper_params = train_agent_utils.initialize_hyperparameters(
            PATHS=paths,
            config=robot_train_params,
            n_envs=config["n_envs"],
        )

        env = vec_env_create(
            env_fn,
            instantiate_train_drl_agents,
            num_robots=robot_train_params["num_robots"],
            num_cpus=cpu_count() - 1,
            num_vec_envs=config["n_envs"],
            task_mode=config["task_mode"],
            PATHS=paths,
            agent_list_kwargs={
                "existing_robots": existing_robots,
                "robot_model": robot_name,
            },
            max_num_moves_per_eps=config["max_num_moves_per_eps"],
        )

        existing_robots += robot_train_params["num_robots"]

        # env = VecNormalize(
        #     env,
        #     training=True,
        #     norm_obs=True,
        #     norm_reward=True,
        #     clip_reward=15,
        #     clip_obs=15,
        # )

        model = choose_agent_model(
            agent_name, paths, robot_train_params, env, hyper_params, config["n_envs"]
        )

        # add configuration for one robot to robots dictionary
        robots[robot_name] = {
            "model": model,
            "env": env,
            "n_envs": config["n_envs"],
            "robot_train_params": robot_train_params,
            "hyper_params": hyper_params,
            "paths": paths,
        }

    # set num of timesteps to be generated
    n_timesteps = 40000000 if config["n_timesteps"] is None else config["n_timesteps"]

    start = time.time()
    try:
        model = robots[robot_name]["model"]
        model.learn(
            total_timesteps=n_timesteps,
            reset_num_timesteps=True,
            # Übergib einfach das dict für den aktuellen roboter
            callback=get_evalcallback(
                eval_config=config["periodic_eval"],
                curriculum_config=config["training_curriculum"],
                stop_training_config=config["stop_training"],
                train_env=robots[robot_name]["env"],
                num_robots=robots[robot_name]["robot_train_params"]["num_robots"],
                num_envs=config["n_envs"],
                task_mode=config["task_mode"],
                PATHS=robots[robot_name]["paths"],
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
    eval_config: dict,
    curriculum_config: dict,
    stop_training_config: dict,
    train_env: VecEnv,
    num_robots: int,
    num_envs: int,
    task_mode: str,
    PATHS: dict,
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
        num_agents=num_robots,
        ns="eval_sim",
        agent_list_fn=instantiate_train_drl_agents,
        max_num_moves_per_eps=eval_config["max_num_moves_per_eps"],
        PATHS=PATHS,
    )

    # eval_env = VecNormalize(
    #     eval_env,
    #     training=False,
    #     norm_obs=True,
    #     norm_reward=False,
    #     clip_reward=15,
    #     clip_obs=3.5,
    # )

    return MarlEvalCallback(
        train_env=train_env,
        eval_env=eval_env,
        num_robots=num_robots,
        n_eval_episodes=eval_config["n_eval_episodes"],
        eval_freq=eval_config["eval_freq"],
        deterministic=True,
        log_path=PATHS["eval"],
        best_model_save_path=PATHS["model"],
        callback_on_eval_end=InitiateNewTrainStage(
            n_envs=num_envs,
            treshhold_type=curriculum_config["threshold_type"],
            upper_threshold=curriculum_config["upper_threshold"],
            lower_threshold=curriculum_config["lower_threshold"],
            task_mode=task_mode,
            verbose=1,
        ),
        callback_on_new_best=StopTrainingOnRewardThreshold(
            treshhold_type=stop_training_config["threshold_type"],
            threshold=stop_training_config["threshold"],
            verbose=1,
        ),
    )


if __name__ == "__main__":
    set_start_method("fork")
    # args, _ = parse_marl_training_args()
    args, _ = parse_training_args()
    # rospy.init_node("train_env", disable_signals=False, anonymous=True)
    main(args)
