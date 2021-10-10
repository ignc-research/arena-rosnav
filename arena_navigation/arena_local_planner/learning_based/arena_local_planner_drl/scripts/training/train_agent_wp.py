import os
import copy
import argparse
import time
from argparse import ArgumentParser
from typing import List

from numpy.lib.utils import info
import rospy
import numpy as np
import pickle

from rl_agent.config import get_cfg, CfgNode
from rl_agent.envs import build_env, build_env_wrapper  # SubprocVecEnv
from rl_agent.model import build_model
from rl_agent.utils.callbacks import TrainStageCallbackWP, StopTrainingOnRewardThreshold
from rl_agent.utils.debug import timeit
from rl_agent.envs import ENV_REGISTRY
from rl_agent.utils.reward import REWARD_REGISTRY
from stable_baselines3.common.vec_env import vec_normalize

from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.evaluation import evaluate_policy

from task_generator import build_task_wrapper
from task_generator.build import build_task
from task_generator.tasks import StopReset
from gym.utils import colorize

def get_default_arg_parser():
    parser = ArgumentParser()
    parser.add_argument(
        '--ns_prefix', help='Namespace prefix',
        default= 'sim_lei'
    )
    parser.add_argument(
        '--pretrained_policy',
        '--pp',
        help="Name to pretrained policy in the pretraining subfolder",
        type=str,
    ),
    parser.add_argument(
        '--rms',
        help='Name to pretrained rms in the pretraining subfolder',
        type=str,
    )
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
        "--load_best_model_for_deploy",
        action='store_true'
    )
    parser.add_argument(
        '--env_range',
        nargs=2,
        help="A tuple of the indices of environments used in training, if second value is -1,then from env_range_start all environments will be used!",
        type=int
    )
    parser.add_argument(
        '--deploy_task_mananer_node_off',
        help='In the deployment mode, whether a independent task node is running, if not a task wrapper will be created',
        type = bool,
        default= True
    )
    parser.add_argument('--debug',
                        help='set the logger level to debug',
                        action='store_true')
    parser.add_argument('opt',
                        help="Overwrite the config loaded from the defaults and config file by providing a list of key and value pairs,\
            In deployment model This feature is disabled.",
                        default=None, nargs=argparse.REMAINDER)
    return parser

def check_args(args,cfg):
    if args.pretrained_policy is None or not cfg.INPUT.NORM:
        assert args.rms is None
    else:
        assert args.rms is not None
        
    
    

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
    else:
        # force to load scenerio task to testing
        cfg.TASK.NAME='ScenerioTask'

    cfg.freeze()
    return cfg


def get_namespaces(args):
    """get a list  of namespaces whose length is the number of environments
    Returns:
        namespaces_list(list): a list of namespaces
    """
    # identical with the one in launch file
    if not args.deploy:
        ns_prefix = args.ns_prefix
        num_envs = rospy.get_param("num_envs")
        assert num_envs>1, "Make sure there are more that 2 simulation environments available since one of them will be used for evalutation"
       
        if args.env_range is None:
            idx_start,idx_end = 1,num_envs
        else:
            idx_start,idx_end = args.env_range
            if idx_end == -1:
                idx_end = num_envs
            assert idx_start<idx_end 
            if idx_end-idx_start+1>=num_envs:
                print(colorize(f"YOU request to use ENV {ns_prefix}_{idx_start} ~ {ns_prefix}{idx_end} for training and evaluation, but the number of the \
                the environments registered on the server is only {num_envs}, make sure you know the exact reason, otherwise there are must be somewhere wrong",'yellow'))
        print(colorize(f"Found {num_envs} ENV {ns_prefix}_{idx_start} ~ {ns_prefix}_{idx_end-1} will be used for training!",'green'))
        
        ns_list = [ns_prefix+'_'+str(i) for i in range(idx_start, idx_end+1)]
    else:
        ns_list = ['']
    return ns_list


def make_envs(cfg, args: argparse.Namespace, namespaces: List[str]):

    if not args.deploy:
        task_wraps = [build_task_wrapper(cfg, ns) for ns in namespaces]
        train_env_wraps = [build_env_wrapper(
            cfg, task, ns, True, args.debug) for task, ns in zip(task_wraps[:-1], namespaces[:-1])]
        if args.debug:
            train_env = DummyVecEnv(train_env_wraps)
        else:
            train_env = SubprocVecEnv(
                train_env_wraps, start_method='fork')
        eval_env = build_env(
        cfg, task_wraps[-1], namespaces[-1], train_mode=False, debug=args.debug)
        output_dir = cfg.OUTPUT_DIR
        info_keywords = ENV_REGISTRY.get(cfg.ENV.NAME).INFO_KEYS + REWARD_REGISTRY.get(cfg.REWARD.RULE_NAME).INFO_KEYS
        eval_env = DummyVecEnv([lambda:Monitor(
            eval_env, output_dir, info_keywords=info_keywords)])
    else:
        if not args.deploy_task_mananer_node_off:
            # if there is a node running which is responsible for manageing the task. we put a None here,
            # so that during the initialization of the environment,nothing will be done for the task wrap,
            # because it's not callable
            task = [None]
        else:
#             if cfg.TASK.NAME == 'StagedRandomTask':
#                 rospy.logwarn("Currently there is not mechanism to load the last stage used for evaluation during the training,\n\
# which is logged in the tensorboard log. Make sure this parameter is synchronized manually by changing \n\
# ‘cfg.EVAL.CURRICULUM.INIT_STAGE_IDX’")
            
            task = build_task(cfg,namespaces[-1])
        train_env = None
        eval_env = build_env(
            cfg, task, namespaces[-1], train_mode=False, debug=args.debug)
        eval_env = DummyVecEnv([lambda:eval_env])
        
    if cfg.INPUT.NORM:
        if not args.deploy:
            train_env = VecNormalize(
                train_env, training=True, norm_obs=True, norm_reward=False, clip_reward=15)

            eval_env = VecNormalize(eval_env, training=False,
                                    norm_obs=True, norm_reward=False, clip_reward=15)

            if args.rms is not None:
                rms_name = args.rms
                if not rms_name.endswith('pkl'):
                    rms_name+='.pkl'
                print("Loading pretrained model for training")
                curr_dir_path = os.path.abspath(os.path.dirname(__file__))
                vec_normalize_obs_rms_file = os.path.join(curr_dir_path,'pretraining_data',rms_name)
                with open(vec_normalize_obs_rms_file,'rb') as f:
                    obs_rms = pickle.load(f)
                    train_env.obs_rms = copy.deepcopy(obs_rms)
                    eval_env.obs_rms = obs_rms
                    print("Loaded rms for the normlized environment for the training and evaluation environment")

        else:
            dir_path  = os.path.split(args.conf_file)[0]

            if args.load_best_model_for_deploy:
                # TODO is the filename correct?
                vec_normalize_file = os.path.join(
                    dir_path, 'vec_normalize.pkl')
            else:
                vec_normalize_file = os.path.join(
                    dir_path, 'final_vec_norm.pkl') 
                if not os.path.isfile(vec_normalize_file):
                    print("load final_vec_norm.pkl failed, try to load another one!")
                    vec_normalize_file = os.path.join(
                        dir_path, 'vec_normalize.pkl')
                

            print("Loading saved model for evaluation")
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

        # thresholds = thresholds[::-1]
        stage_idx_range = [0,len(cfg.EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE)-1]
        trainstage_callback = TrainStageCallbackWP(
            namespaces,
            cfg.TASK.NAME,
            *thresholds,
            stage_idx_range = stage_idx_range,
            init_stage_idx=cfg.EVAL.CURRICULUM.INIT_STAGE_IDX,
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
    saved_model_file_names = ["best_model.zip","final_model.zip"]
    if args.load_best_model_for_deploy:     
        model_file_idx = 0
    else:
        model_file_idx = 1
    model_file_found = False
    for _ in range(2):
        dir_path  = os.path.split(args.conf_file)[0]
        model_file = os.path.join(dir_path, saved_model_file_names[model_file_idx])
        if not os.path.exists(model_file):
            print(f"Model file {model_file} not found")
            model_file_idx  = 1-model_file_idx
        else:
            model_file_found = True
            break 
    if not model_file_found:
        raise FileNotFoundError("You know the reason")

    if cfg.MODEL.NAME == 'PPO':
        from stable_baselines3 import PPO
        model = PPO.load(model_file, env)
        print(f"loaded {model_file}")
    else:
        raise NotImplementedError()
    return model


def deploy_run(model, env):
    try:
        evaluate_policy(
                model=model,
                env=env,
                n_eval_episodes=1000,
                deterministic=True
            )
    except StopReset:
        print("ALL scenrios haved be evaluated, program exit!")

def load_pretrain_policy(cfg,model,pretrain_dirpath):  
    pretrained_policy_file_name = "pretrain_policy.pkl"
    pretrained_policy_file_path = os.path.join(pretrain_dirpath, pretrained_policy_file_name)
    import pickle
    with open(pretrain_dirpath,'rb') as f:
        pretrained_policy = pickle.load(f)
    model.policy = pretrained_policy
    return model



def main():
    parser = get_default_arg_parser()
    args = parser.parse_args()
    cfg = setup_config(args)
    check_args(args,cfg)
    namespaces = get_namespaces(args)
    training_env, eval_env = make_envs(cfg, args, namespaces)


    if not args.deploy:
        try:
            model = build_model(cfg, training_env,
                                tensorboard_log=cfg.OUTPUT_DIR, debug=args.debug)
            eval_callback = build_eval_callback(
                cfg, namespaces, eval_env=eval_env, train_env=training_env)
            if args.pretrained_policy is not None:
                pretrained_policy_name = args.pretrained_policy
                if not pretrained_policy_name.endswith('.pkl'):
                    pretrained_policy_name += '.pkl'
                pretrained_policy_path = os.path.join(os.path.dirname(__file__),'pretraining_data',pretrained_policy_name)
                with open(pretrained_policy_path,'rb') as f:
                    pretrained_policy = pickle.load(f)
                    model.policy = pretrained_policy
                    print(f"Loaded pretrained policy {pretrained_policy_name} ")
            model.learn(
                total_timesteps=cfg.TRAINING.N_TIMESTEPS, callback=eval_callback, reset_num_timesteps=True)
        except KeyboardInterrupt:
            print("User request to terminate the program...")
        finally:
            model.save(cfg.OUTPUT_DIR+"/final_model.zip")
            print(f"Successfully saved the model to {cfg.OUTPUT_DIR+'/final_model.zip'}")
            if cfg.INPUT.NORM:
                training_env.save(cfg.OUTPUT_DIR+"/final_vec_norm.pkl")
                print(f"Successfully saved the normalized environment to {cfg.OUTPUT_DIR+'/final_model.pkl'}")
    else:
        model = load_model(cfg, args, eval_env)
        deploy_run(model, eval_env)


if __name__ == "__main__":
    main()


# TO run in deployment mode
# 1. roslaunch arena_bringup start_training.launch num_envs:=1 train_mode:=false map_folder_name:=map1
# 2. python deplpyment/action_publisher.py
# 3. run this scripy with python run_agent_v2.py  --conf_file=${PATHTOHyperparams.yaml} --deploy  
