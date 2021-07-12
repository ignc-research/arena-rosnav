import os
import copy
import argparse
import time
from argparse import ArgumentParser
from typing import List
import rospy
import numpy as np

from rl_agent.config import get_cfg, CfgNode
from rl_agent.envs import build_env, build_env_wrapper  # SubprocVecEnv
from rl_agent.model import build_model
from rl_agent.utils.callbacks import TrainStageCallbackWP, StopTrainingOnRewardThreshold
from rl_agent.utils.debug import timeit
from stable_baselines3.common.vec_env import vec_normalize

from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from stable_baselines3.common.callbacks import EvalCallback

from task_generator import build_task_wrapper
from gym.utils import colorize

def get_default_arg_parser():
    parser = ArgumentParser()
    parser.add_argument(
        '--conf_file',
        help="The path of the config file. In training mode this is optional. but in deployment mode this is mandatory",
        type=str)
    parser.add_argument(
        '--deploy',
        help='Is the script running at training mode or in deployment mode',
        action='store_true',
    ),
    parser.add_argument(
        '--env_range',
        nargs=2,
        help="A tuple of the indices of environments used in training, if second value is -1,then from env_range_start all environments will be used!",
        type=int
    )
    parser.add_argument(
        '--deploy_task_mananer_node_off',
        help='In the deployment mode, whether a independent task node is running, if not a task wrapper will be created',
        action='store_true'
    )
    parser.add_argument('--debug',
                        help='set the logger level to debug',
                        action='store_true')
    parser.add_argument('opt',
                        help="Overwrite the config loaded from the defaults and config file by providing a list of key and value pairs,\
            In deployment model This feature is disabled.",
                        default=None, nargs=argparse.REMAINDER)
    return parser


def setup_config(args):
    # get the global default config and merge it with the optional config file and arguments
    cfg = get_cfg(is_cfg_for_waypoint=True)
    if args.deploy:
        assert args.conf_file is not None, "In deployment mode, config file must be given."

    if args.conf_file is not None:

        if args.deploy:
            cfg.OUTPUT_DIR = None
        cfg.merge_from_file(args.conf_file)

    if not args.deploy:
        cfg.merge_from_list(args.opt)
        # set current output dir
        # currently based on the feature extractor name and time
        import datetime
        curr_time_str = datetime.datetime.now().strftime("_%Y_%m_%d_%H_%M_%S")
        feature_extractor_name = cfg.NET_ARCH.FEATURE_EXTRACTOR.NAME
        output_dir = os.path.join(
            cfg.OUTPUT_DIR_ROOT, feature_extractor_name+curr_time_str)
        os.makedirs(output_dir)
        cfg.OUTPUT_DIR = output_dir
        print("Training logs will be written to the directory: ")
        print(f"\t{cfg.OUTPUT_DIR}")
        # freeze the config and save it.
        # TODO  we know there are some settings stored in some other places which is not trival for the training
        # it would be a good idea to make a snapshot of them and store them in the output folder
        with open(os.path.join(cfg.OUTPUT_DIR, "Hyperparams.yaml"), "w") as f:
            cfg.dump(stream=f)

    cfg.freeze()
    return cfg


def get_namespaces(args):
    """get a list  of namespaces whose length is the number of environments
    Returns:
        namespaces_list(list): a list of namespaces
    """
    # identical with the one in launch file
    ns_prefix = "sim"
    num_envs = rospy.get_param("num_envs")
    if args.env_range is None:
        idx_start,idx_end = 1,num_envs
    else:
        idx_start,idx_end = args.env_range
        if idx_end == -1:
            idx_end = num_envs
        assert idx_start<idx_end and idx_end<=num_envs
    
    ns_list = [ns_prefix+'_'+str(i) for i in range(idx_start, idx_end+1)]

    if not args.deploy:
        assert num_envs>1, "Make sure there are more that 2 simulation environments available since one of them will be used for evalutation"
        print(colorize(f"Found {num_envs} ENV {ns_prefix}{idx_start} - {ns_prefix}{idx_end-1} will be used for training!",'green'))
    elif num_envs > 1:
        rospy.logwarn(
            f"Found {num_envs} running at backend, but in deployment mode only one of them will be used!")
        # return one namespace
        ns_list = ns_list[:2]
    return ns_list


def make_envs(cfg, args: argparse.Namespace, namespaces: List[str]):

    task_wraps = [build_task_wrapper(cfg, ns) for ns in namespaces]
    if not args.deploy:
        train_env_wraps = [build_env_wrapper(
            cfg, task, ns, True, args.debug) for task, ns in zip(task_wraps[:-1], namespaces[:-1])]
        if args.debug:
            train_env = DummyVecEnv(train_env_wraps)
        else:
            train_env = SubprocVecEnv(
                train_env_wraps, start_method='fork')
    else:
        if not args.deploy_task_mananer_node_off:
            # if there is a node running which is responsible for manageing the task. we put a None here,
            # so that during the initialization of the environment,nothing will be done for the task wrap,
            # because it's not callable
            task_wraps = [None]
        else:
            if cfg.TASK.NAME == 'StagedRandomTask':
                rospy.logwarn("Currently there is not mechanism to load the last stage used for evaluation during the training,\n\
which is logged in the tensorboard log. Make sure this parameter is synchronized manually by changing \n\
‘cfg.EVAL.CURRICULUM.INIT_STAGE_IDX’")

        train_env = None

    eval_env = build_env(
        cfg, task_wraps[-1], namespaces[-1], train_mode=False, debug=args.debug)

    if not args.deploy:
        output_dir = cfg.OUTPUT_DIR
        eval_env = DummyVecEnv([lambda:Monitor(
            eval_env, output_dir, info_keywords=("done_reason", "is_success"))])
    else:
        eval_env = DummyVecEnv([lambda:eval_env])

    if cfg.INPUT.NORM:
        if not args.deploy:
            train_env = VecNormalize(
                train_env, training=True, norm_obs=True, norm_reward=False, clip_reward=15)

            eval_env = VecNormalize(eval_env, training=False,
                                    norm_obs=True, norm_reward=False, clip_reward=15)
        else:
            dir_path  = os.path.split(args.conf_file)[0]
            vec_normalize_file = os.path.join(
                dir_path, 'vec_normalize.pkl')
        
            assert os.path.isfile(
                vec_normalize_file), f"{vec_normalize_file} does't exist"
            eval_env = VecNormalize.load(vec_normalize_file, eval_env)
            # in case the loaded one's training is True
            eval_env.training = False

    return train_env, eval_env


def build_eval_callback(cfg, namespaces: List[str], eval_env, train_env):
    if not cfg.EVAL.STOP_TRAINING_ON_REWARD:
        stop_training_callback = None
    else:
        stop_training_callback = StopTrainingOnRewardThreshold(
            cfg.EVAL.STOP_TRAINING_ON_REWARD_THRESHOLD, verbose=1)

    if cfg.TASK.NAME == "StagedRandomTask":
        
        thresholds = cfg.EVAL.CURRICULUM.THRESHOLD_RANGE

        thresholds = thresholds[::-1]
        trainstage_callback = TrainStageCallbackWP(
            namespaces,
            cfg.TASK.NAME,
            *thresholds,
            verbose=1)
    else:
        trainstage_callback = None

    eval_callback = EvalCallback(
        eval_env,
        train_env,
        n_eval_episodes=cfg.EVAL.N_EVAL_EPISODES,
        eval_freq=cfg.EVAL.EVAL_FREQ,
        log_path=cfg.OUTPUT_DIR,
        best_model_save_path=cfg.OUTPUT_DIR,
        deterministic=True,
        callback_on_eval_end=trainstage_callback,
        callback_on_new_best=stop_training_callback)

    return eval_callback


def load_model(cfg, args, env):
    dir_path  = os.path.split(args.conf_file)[0]
    model_file = os.path.join(dir_path, 'best_model.zip')
    if cfg.MODEL.NAME == 'PPO':
        from stable_baselines3 import PPO
        model = PPO.load(model_file, env)
    else:
        raise NotImplementedError()
    return model


def deploy_run(cfg: CfgNode, args, model, env):
    env.reset()
    first_obs = True
    # iterate through each scenario max_repeat times
    while True:
        if first_obs:
            # send action 'stand still' in order to get first obs
            if cfg.ROBOT.IS_ACTION_DISCRETE:
                obs, rewards, dones, info = env.step([6])
            else:
                obs, rewards, dones, info = env.step([[0.0, 0.0]])
            first_obs = False
            cum_reward = 0.0

        # timer = time.time()
        action, _ = model.predict(obs, deterministic=True)
        # print(f"Action predict time: {(time.time()-timer)*2.5} (sim time)")

        # clip action
        if not cfg.ROBOT.IS_ACTION_DISCRETE:
            action = np.maximum(
                np.minimum(model.action_space.high, action), model.action_space.low)

        # apply action
        obs, rewards, done, info = env.step(action)

        cum_reward += rewards

        if done:
            if info[0]['done_reason'] == 0:
                done_reason = "exceeded max steps"
            elif info[0]['done_reason'] == 1:
                done_reason = "collision"
            else:
                done_reason = "goal reached"
                print("Episode finished with reward of %f (finish reason: %s)" % (
                    cum_reward, done_reason))
            env.reset()
            first_obs = True
        time.sleep(0.001)
        if rospy.is_shutdown():
            print('shutdown')
            break


def main():
    parser = get_default_arg_parser()
    args = parser.parse_args()
    cfg = setup_config(args)
    namespaces = get_namespaces(args)
    training_env, eval_env = make_envs(cfg, args, namespaces)

    if not args.deploy:
        model = build_model(cfg, training_env,
                            tensorboard_log=cfg.OUTPUT_DIR, debug=args.debug)
        eval_callback = build_eval_callback(
            cfg, namespaces, eval_env=eval_env, train_env=training_env)
        model.learn(
            total_timesteps=cfg.TRAINING.N_TIMESTEPS, callback=eval_callback, reset_num_timesteps=True)
    else:
        model = load_model(cfg, args, eval_env)
        deploy_run(cfg, args, model, eval_env)


if __name__ == "__main__":
    main()


# TO run in deployment mode
# 1. roslaunch arena_bringup start_training.launch num_envs:=1 train_mode:=false map_folder_name:=map1
# 2. python deplpyment/action_publisher.py
# 3. run this scripy with python run_agent_v2.py  --conf_file=${PATHTOHyperparams.yaml} --deploy --deploy_task_mananer_node_off  
