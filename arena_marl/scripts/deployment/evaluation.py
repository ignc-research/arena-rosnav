from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import numpy as np
from stable_baselines3.common import base_class
import supersuit.vector.sb3_vector_wrapper as sb3vw


def evaluate_policy(
    model: "base_class.BaseAlgorithm",
    # env: Union[sb3vw.SB3VecEnvWrapper, VecEnv],
    num_robots: int,
    env: sb3vw.SB3VecEnvWrapper,
    n_eval_episodes: int = 10,
    deterministic: bool = True,
    render: bool = False,
    callback: Optional[Callable[[Dict[str, Any], Dict[str, Any]], None]] = None,
    reward_threshold: Optional[float] = None,
    return_episode_rewards: bool = False,
    warn: bool = True,
) -> Union[Tuple[float, float], Tuple[List[float], List[int]]]:
    """
    Runs policy for ``n_eval_episodes`` episodes and returns average reward.
    This is made to work only with one env.

    .. note::
        If environment has not been wrapped with ``Monitor`` wrapper, reward and
        episode lengths are counted as it appears with ``env.step`` calls. If
        the environment contains wrappers that modify rewards or episode lengths
        (e.g. reward scaling, early episode reset), these will affect the evaluation
        results as well. You can avoid this by wrapping environment with ``Monitor``
        wrapper before anything else.

    :param model: The RL agent you want to evaluate.
    :param env: The gym environment. In the case of a ``VecEnv``
        this must contain only one environment.
    :param n_eval_episodes: Number of episode to evaluate the agent
    :param deterministic: Whether to use deterministic or stochastic actions
    :param render: Whether to render the environment or not
    :param callback: callback function to do additional checks,
        called after each step. Gets locals() and globals() passed as parameters.
    :param reward_threshold: Minimum expected reward per episode,
        this will raise an error if the performance is not met
    :param return_episode_rewards: If True, a list of rewards and episode lengths
        per episode will be returned instead of the mean.
    :param warn: If True (default), warns user about lack of a Monitor wrapper in the
        evaluation environment.
    :return: Mean reward per episode, std of reward per episode.
        Returns ([float], [int]) when ``return_episode_rewards`` is True, first
        list containing per-episode rewards and second containing per-episode lengths
        (in number of steps).
    """
    is_monitor_wrapped = False
    # Avoid circular import
    from stable_baselines3.common.env_util import is_wrapped
    from stable_baselines3.common.monitor import Monitor

    not_reseted = True
    # Avoid double reset, as VecEnv are reset automatically.
    if not_reseted:
        # Observations dictionary in {_agent name_: _respective observations_}
        obs = env.reset()
        not_reseted = False

    agents = [a for a, _ in obs.items()]
    episode_rewards = {a: [None] for a in agents}
    episode_lengths = []

    default_dones = {a: False for a in agents}
    default_states = {a: None for a in agents}
    default_actions = {a: None for a in agents}
    default_episode_reward = {a: None for a in agents}
    while len(episode_rewards) < n_eval_episodes:
        # Number of loops here might differ from true episodes
        # played, if underlying wrappers modify episode lengths.
        # Avoid double reset, as VecEnv are reset automatically.
        if not_reseted:
            # Observations dictionary in {_agent name_: _respective observations_}
            obs = env.reset()
            not_reseted = False
        dones = default_dones.copy()
        states = default_states.copy()
        actions = default_actions.copy()
        episode_reward = default_episode_reward.copy()
        episode_length = 0
        while not dones:
            # Get actions and states from each agent
            for agent, state in states.items():
                actions[agent], states[agent] = model.predict(
                    obs[agent], state, deterministic=deterministic
                )

            obs, rewards, dones, infos = env.step(actions)
            for agent, reward in zip(agents, rewards.values()):
                episode_reward[agent] += reward

            if callback is not None:
                callback(locals(), globals())

            episode_length += 1
            if render:
                env.render()

        # if is_monitor_wrapped:
        #     # Do not trust "done" with episode endings.
        #     # Remove vecenv stacking (if any)
        #     # if isinstance(env, VecEnv):
        #     #     info = info[0]
        #     if "episode" in info.keys():
        #         # Monitor wrapper includes "episode" key in info if environment
        #         # has been wrapped with it. Use those rewards instead.
        #         episode_rewards.append(info["episode"]["r"])
        #         episode_lengths.append(info["episode"]["l"])
        # else:
        for agent, reward in episode_reward.items():
            episode_rewards[agent].append(reward)
        episode_lengths.append(episode_length)

    mean_rewards = {agent: np.mean(episode_rewards[agent]) for agent in agents}
    std_rewards = {agent: np.std(episode_rewards[agent]) for agent in agents}
    if reward_threshold is not None:
        assert min(mean_rewards.values()) > reward_threshold, (
            "Atleast one mean reward below threshold: "
            f"{min(mean_rewards.values()):.2f} < {reward_threshold:.2f}"
        )
    if return_episode_rewards:
        return episode_rewards, episode_lengths
    return mean_rewards, std_rewards
