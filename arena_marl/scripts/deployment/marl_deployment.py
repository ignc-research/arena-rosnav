from multiprocessing import cpu_count
import os


from arena_marl.marl_agent.utils.utils import (
    instantiate_deploy_drl_agents,
    get_hyperparameter_file,
    instantiate_train_drl_agents,
)

from arena_marl.scripts.deployment.evaluation import evaluate_policy

from marl_tools.train_agent_utils import (
    load_config,
)
from marl_tools import train_agent_utils

import rospy

from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO
from arena_marl.marl_agent.utils.supersuit_utils import (
    vec_env_create,
)
from arena_marl.marl_agent.envs.pettingzoo_env import env_fn


from tools.argsparser import parse_training_args


def main(args):
    # load configuration
    config = load_config(args.config)

    # set debug_mode
    rospy.set_param("debug_mode", config["debug_mode"])

    # load agents
    assert len(config["robots"]) == 1, "Only one robot type is supported"

    robots, existing_robots = {}, 0

    # loop over all robots in config
    for robot_name, robot_train_params in config["robots"].items():
        # Get the Base Agent instance(s)
        robots[robot_name] = {
            "agents": instantiate_train_drl_agents(
                num_robots=robot_train_params["num_robots"],
                robot_model=robot_name,
                hyperparameter_path=get_hyperparameter_file(
                    robot_train_params["hyperparameter_file"]
                ),
            )
        }
        existing_robots += robot_train_params["num_robots"]

        # Define paths. If the agent is a SARL agent, MARL_DIR is None.
        MARL_DIR = ""
        paths = train_agent_utils.get_paths(
            MARL_DIR,
            robot_name,
            robot_train_params["resume"].split("/")[-1],
            robot_train_params,
            config["training_curriculum"]["training_curriculum_file"],
            config["eval_log"],
            config["tb"],
        )

        # Create env and store in dict
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
        robots[robot_name] = {"env": env}

        # Load PPO model and store in dict
        model = PPO.load(
            os.path.join(robot_train_params["resume"], "best_model.zip"), env
        )
        robots[robot_name]["model"] = model

    # Evaluate the policy for one robot type!
    # Integration of multiple robot types is not yet implemented
    mean_rewards, std_rewards = evaluate_policy(
        model=robots[robot_name]["model"],
        num_robots=robot_train_params["num_robots"],
        env=robots[robot_name]["env"],
        n_eval_episodes=config["periodic_eval"]["n_eval_episodes"],
        return_episode_rewards=False,
    )

    print("Mean rewards: {}".format(mean_rewards))
    print("Std rewards: {}".format(std_rewards))


if __name__ == "__main__":
    args, _ = parse_training_args()
    main(args)
