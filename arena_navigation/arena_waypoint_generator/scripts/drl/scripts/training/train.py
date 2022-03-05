#!/usr/bin/env python
from typing import Type, Union

import os, sys, rospy, time, gym

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.callbacks import (
    EvalCallback,
    StopTrainingOnRewardThreshold,
)
from stable_baselines3.common.policies import ActorCriticPolicy, BasePolicy
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed


#from arena_navigation.arena_waypoint_generator.scripts.drl.rl_agent.envs.subgoal_env import (Subgoal_env)

from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.vec_env.base_vec_env import VecEnv

from arena_navigation.arena_waypoint_generator.scripts.drl.tools.argsparser import parse_training_args
from arena_navigation.arena_waypoint_generator.scripts.drl.tools.custom_mlp_utils import *
from arena_navigation.arena_waypoint_generator.scripts.drl.tools.train_agent_utils import *
from arena_navigation.arena_waypoint_generator.scripts.drl.tools.staged_train_callback import InitiateNewTrainStage

def main():
    
    rospy.init_node("debug_node", disable_signals=False)

    args, _ = parse_training_args()

    AGENT_NAME = get_agent_name(args)
    PATHS = get_paths(AGENT_NAME, args)

    ns_for_nodes = True #"/single_env" not in rospy.get_param_names()

    # check if simulations are booted
    wait_for_nodes(with_ns=ns_for_nodes, n_envs=args.n_envs, timeout=5)

    # initialize hyperparameters (save to/ load from json)
    params = initialize_hyperparameters(
        PATHS=PATHS,
        load_target=args.load,
        config_name=args.config,
        n_envs=args.n_envs,
    )

    if not args.debug and ns_for_nodes:
        env = SubprocVecEnv(
            [
                make_envs(args, ns_for_nodes, i, params=params, PATHS=PATHS)
                for i in range(args.n_envs)
            ],
            start_method="fork",
        )
    else:
        env = DummyVecEnv(
            [
                make_envs(args, ns_for_nodes, i, params=params, PATHS=PATHS)
                for i in range(args.n_envs)
            ]
        )
        
    trainstage_cb = InitiateNewTrainStage(
        n_envs=args.n_envs,
        treshhold_type="succ",
        upper_threshold=0.9,
        lower_threshold=0.7,
        task_mode=params["task_mode"],
        verbose=1,
    )

    stoptraining_cb = StopTrainingOnRewardThreshold(
        treshhold_type="succ", threshold=0.95, verbose=1
    )

    if ns_for_nodes:
        eval_env = DummyVecEnv(
            [
                make_envs(
                    args,
                    ns_for_nodes,
                    0,
                    params=params,
                    PATHS=PATHS,
                    train=False,
                )
            ]
        )
    else:
        eval_env = env

    env, eval_env = load_vec_normalize(params, PATHS, env, eval_env)

    eval_cb = EvalCallback(
        eval_env=eval_env,
        train_env=env,
        n_eval_episodes=100,
        eval_freq=22500,
        log_path=PATHS["eval"],
        best_model_save_path=PATHS["model"],
        deterministic=True,
        callback_on_eval_end=trainstage_cb,
        callback_on_new_best=stoptraining_cb,
    )

    model = PPO(
            "MlpPolicy",
            env,
            policy_kwargs=dict(
                net_arch=args.net_arch, activation_fn=get_act_fn(args.act_fn)
            ),
            gamma=params["gamma"],
            n_steps=params["n_steps"],
            ent_coef=params["ent_coef"],
            learning_rate=params["learning_rate"],
            vf_coef=params["vf_coef"],
            max_grad_norm=params["max_grad_norm"],
            gae_lambda=params["gae_lambda"],
            batch_size=params["m_batch_size"],
            n_epochs=params["n_epochs"],
            clip_range=params["clip_range"],
            tensorboard_log=PATHS["tb"],
            verbose=1,
        )

    # set num of timesteps to be generated
    n_timesteps = 40_000_000 if args.n is None else args.n
    # start training
    start = time.time()
    try:
        model.learn(
            total_timesteps=n_timesteps,
            callback=eval_cb,
            reset_num_timesteps=True,
        )
    except KeyboardInterrupt:
        print("KeyboardInterrupt..")

    model.env.close()
    print(f"Time passed: {time.time()-start}s")
    print("Training script will be terminated")
    sys.exit()
    


if __name__ == "__main__":
    main()