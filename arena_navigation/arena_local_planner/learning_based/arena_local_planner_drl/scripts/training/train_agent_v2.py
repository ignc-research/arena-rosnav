import argparse
from argparse import ArgumentParser
from typing import List
import os
import copy
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from rl_agent.envs import build_env
from rl_agent.model.build import build_model
from task_generator import build_tasks
from rl_agent.config import get_cfg
import rospy
from tools.argsparser import parse_run_agent_args
from rl_agent.utils.debug import timeit


def get_default_arg_parser():
    parser = ArgumentParser()
    parser.add_argument('--conf_file',help="The path of the config file.",type=str)
    parser.add_argument('--debug',action='store_true')
    parser.add_argument('opt',help="Overwrite the config loaded from the \
        defaults and config file by providing a list of key and value pairs",
        default=None,nargs= argparse.REMAINDER)
    return parser

def setup_config(args):
    cfg = get_cfg()
    if args.conf_file is not None:
        cfg.merge_from_file(args.conf_file)
    cfg.merge_from_list(args.opt)

    # set current output dir
    # currently based on the feature extractor name and time
    import datetime
    curr_time_str = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    feature_extractor_name = cfg.NET_ARCH.FEATURE_EXTRACTOR.NAME
    output_dir  = os.path.join(cfg.OUTPUT_DIR_ROOT,feature_extractor_name+curr_time_str)
    os.makedirs(output_dir)
    cfg.OUTPUT_DIR = output_dir
    
    cfg.freeze()
    return cfg

def get_namespaces():
    """get a list  of namespaces whose length is the number of environments
    Returns:
        namespaces_list(list): a list of namespaces
    """
    # identical with the one in launch file
    ns_prefix = "sim" 
    num_envs= rospy.get_param("num_envs")
    ns_list = [ns_prefix+'_'+str(i) for i in range(1,num_envs+1)]
    return ns_list


def make_envs(cfg,args:argparse.Namespace,namespaces:List[str]):

    def build_env_wrapper(task,ns,train_mode,debug):
        def wrappee():
            env = build_env(cfg,task,ns,train_mode=train_mode,debug=debug)
            return env
        return wrappee




    training_env_list = []
    tasks = build_tasks(cfg,namespaces)

    if args.debug:
        VecENV = DummyVecEnv
    else:
        VecENV = SubprocVecEnv
    training_env = VecENV([build_env_wrapper(task,ns,True,args.debug) for task,ns in zip(tasks,namespaces)],
        start_method='fork')
    # evaluation environment on flatland is the first environment
    output_dir = cfg.OUTPUT_DIR
    eval_env = lambda: Monitor(build_env(cfg,tasks[0],namespaces[0],train_mode=False,debug=args.debug),
        output_dir,
        info_keywords=("done_reason", "is_success"))
    eval_env = VecENV([eval_env],start_method='fork')
    if cfg.INPUT.NORM:
        training_env  = VecNormalize(training_env,training=True,norm_obs=True,norm_reward=False,clip_reward=15)

        eval_env = VecNormalize(eval_env,training=False,norm_obs=True,norm_reward=False,clip_reward=15)
    return training_env,eval_env

def build_eval_callback(cfg,eval_env):
    if cfg.EVAL.STOP_TRAINING_ON_REWARD is None:
        stop_training_callback = None
    else:
        stop_training_callback = StopTrainingOnRewardThreshold(cfg.EVAL.STOP_TRAINING_ON_REWARD,verbose=1)

    eval_callback = EvalCallback(
        eval_env, 
        n_eval_episodes=cfg.EVAL.N_EVAL_EPISODES,
        eval_freq=cfg.EVAL.EVAL_FREQ,
        log_path=cfg.OUTPUT_DIR,
        best_model_save_path=cfg.OUTPUT_DIR, 
        deterministic=True,
        callback_on_new_best=stop_training_callback)
    return eval_callback


@timeit
def main():
    parser = get_default_arg_parser()
    args = parser.parse_args()
    cfg = setup_config(args)
    namespaces = get_namespaces()
    training_env,eval_env = make_envs(cfg,args,namespaces)
    model = build_model(cfg,training_env,tensorboard_log=cfg.OUTPUT_DIR)
    eval_callback = build_eval_callback(cfg,eval_env=eval_env)
    model.learn(
        total_timesteps = cfg.TRAINING.N_TIMESTEPS, callback=eval_callback, reset_num_timesteps=True)
    

if __name__ == "__main__":
    main()







    








    
