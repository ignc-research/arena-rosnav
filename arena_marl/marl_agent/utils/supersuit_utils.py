from functools import partial
from typing import Callable

import numpy as np
import rospy

# from stable_baselines3.common.vec_env import VecNormalize
from supersuit.vector import ConcatVecEnv, MarkovVectorEnv


class MarkovVectorEnv_patched(MarkovVectorEnv):
    """Patched environment wrapper which creates the correct API for vector environments. Dones for dead agents are returned as True instead as False."""

    def step(self, actions):
        agent_set = set(self.par_env.agents)
        act_dict = {
            agent: actions[i]
            for i, agent in enumerate(self.par_env.possible_agents)
            if agent in agent_set
        }
        observations, rewards, dones, infos = self.par_env.step(act_dict)

        # adds last observation to info where user can get it
        if all(dones.values()):
            for agent, obs in observations.items():
                infos[agent]["terminal_observation"] = obs

        rews = np.array(
            [rewards.get(agent, 0) for agent in self.par_env.possible_agents],
            dtype=np.float32,
        )
        # we changed the default value to true instead of false
        dns = np.array(
            [dones.get(agent, True) for agent in self.par_env.possible_agents],
            dtype=np.uint8,
        )
        infs = [infos.get(agent, {}) for agent in self.par_env.possible_agents]

        if all(dones.values()):
            observations = self.reset()
        else:
            observations = self.concat_obs(observations)
        assert (
            self.black_death or self.par_env.agents == self.par_env.possible_agents
        ), "MarkovVectorEnv does not support environments with varying numbers of active agents unless black_death is set to True"
        return observations, rews, dns, infs


def vec_env_create(
    env_fn: Callable,
    agent_list_fn: Callable,
    num_robots: int,
    task_mode: str,
    num_cpus: int,
    num_vec_envs: int,
    PATHS: dict,
    agent_list_kwargs: dict,
    max_num_moves_per_eps: int,
):
    """Function which vectorizes a given environment function in multiple parallel environments.

    Args:
        env_fn (Callable): Function that initializes an environment with wrappers
        agent_list_fn (Callable): Object containing the program arguments
        robot_model (str): Name of the robot model
        num_robots (int): Number of robots in the environment
        task_mode (str): Navigation task mode
        num_cpus (int): Maximal number of CPUs to use (Currently only process is used anyhow)
        num_vec_envs (int): Number of parallel environments to spawn
        PATHS (dict): Dictionary which holds hyperparameters for the experiment

    Returns:
        SB3VecEnvWrapper: Vectorized environments following the SB3 VecEnv API. Each each robot in an environment \
            poses as an environment in the vector.
    """
    import supersuit.vector.sb3_vector_wrapper as sb3vw

    env_list_fns = [
        partial(
            env_fn,
            ns=f"sim_{i}",
            num_agents=num_robots,
            task_mode=task_mode,
            agent_list_fn=agent_list_fn,
            PATHS=PATHS,
            agent_list_kwargs=agent_list_kwargs,
            max_num_moves_per_eps=max_num_moves_per_eps,
        )
        for i in range(1, num_vec_envs + 1)
    ]
    env = env_list_fns[0]()
    action_space = env.action_space
    observation_space = env.observation_space
    metadata = env.metadata

    num_cpus = min(num_cpus, num_vec_envs)
    rospy.init_node("train_env", disable_signals=False, anonymous=True)
    vec_env = ConcatVecEnv(env_list_fns, observation_space, action_space)
    return sb3vw.SB3VecEnvWrapper(vec_env)
