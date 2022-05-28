import numpy as np
import rospy
import rospkg
import os

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import MarlEvalCallback
from supersuit import pettingzoo_env_to_vec_env_v0, black_death_v2
from supersuit.vector.sb3_vector_wrapper import SB3VecEnvWrapper

rospy.set_param("/MARL", True)
rospy.set_param("/num_robots", 8)

from rl_agent.training_agent_wrapper import TrainingDRLAgent
from scripts.deployment.drl_agent_node import DeploymentDRLAgent
from rl_agent.envs.pettingzoo_env import FlatlandPettingZooEnv

from nav_msgs.srv import GetMap

DEFAULT_HYPERPARAMETER = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    "hyperparameters",
    "default.json",
)
DEFAULT_ACTION_SPACE = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    "default_settings.yaml",
)


def instantiate_drl_agents(
    num_robots: int = 1,
    ns: str = None,
    robot_name_prefix: str = "robot",
    hyperparameter_path: str = DEFAULT_HYPERPARAMETER,
    action_space_path: str = DEFAULT_ACTION_SPACE,
) -> list:
    return [
        TrainingDRLAgent(
            ns=ns,
            robot_name=robot_name_prefix + str(i + 1),
            hyperparameter_path=hyperparameter_path,
            action_space_path=action_space_path,
        )
        for i in range(num_robots)
    ]


def main():
    # rospy.init_node(f"USER_NODE", anonymous=True)

    # agent_list = instantiate_drl_agents(
    #     num_robots=8,
    #     ns="sim_1",
    #     robot_name_prefix=rospy.get_param("base_robot_name", default="robot"),
    # )
    NUM_ROBOTS = 8

    env = SB3VecEnvWrapper(
        pettingzoo_env_to_vec_env_v0(
            black_death_v2(
                FlatlandPettingZooEnv(
                    num_agents=NUM_ROBOTS,
                    ns="sim_1",
                    agent_list_fn=instantiate_drl_agents,
                    max_num_moves_per_eps=2000,
                )
            )
        )
    )
    obs = env.reset()

    # AGENT = DeploymentDRLAgent(
    #     agent_name="rule_04", ns="sim_1", robot_name="test1"
    # )

    model = PPO("MlpPolicy", env)
    model.learn(
        total_timesteps=200000,
        callbacks=get_evalcallback(
            num_robots=NUM_ROBOTS,
        ),
        reset_num_timesteps=True,
    )
    exit()
    agent_names = env.agents
    for _ in range(100000000):
        if not agent_names:
            agent_names = env.possible_agents[:]
            obs = env.reset()

        actions = {agent: AGENT.get_action(obs[agent]) for agent in agent_names}
        obs, rewards, dones, infos = env.step(actions)

        done_agents = [k for k, v in dones.items() if v]
        for agent in done_agents:
            agent_names.remove(agent)

        if done_agents:
            for agent in done_agents:
                if infos[agent]["is_success"]:
                    print(f"{agent}: finito")


def get_evalcallback(num_robots: int) -> MarlEvalCallback:
    eval_env = FlatlandPettingZooEnv(
        num_agents=num_robots,
        ns="eval_sim",
        agent_list_fn=instantiate_drl_agents,
        max_num_moves_per_eps=2000,
    )

    return MarlEvalCallback(
        eval_env,
        num_robots,
        n_eval_episodes=40,
        eval_freq=1,
        deterministic=True,
    )


if __name__ == "__main__":
    main()
