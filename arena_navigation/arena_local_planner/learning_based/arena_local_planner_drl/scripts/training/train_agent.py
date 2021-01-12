import os
import rospy
from datetime import datetime as dt

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback

from task_generator.task_generator.tasks import get_predefined_task
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.scripts.custom_policy import *
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.envs.flatland_gym_env import FlatlandEnv
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.argsparser import parse_training_args
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.train_agent_utils import *
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.custom_mlp_args_utils import *


###HYPERPARAMETER###
robot = "myrobot"
gamma = 0.99
n_steps = 128
ent_coef = 0.01
learning_rate = 2.5e-4
vf_coef = 0.5
max_grad_norm = 0.5
gae_lambda = 0.95
batch_size = 64
n_epochs = 4
clip_range = 0.2
discrete_action_space = True
####################

def get_agent_name(args):
    """ get agent name to save/load to/from file system

    Example names:
    "MLP_B_64-64_P_32-32_V_32-32_relu_2021_01_07__10_32"
    "DRL_LOCAL_PLANNER_2021_01_08__7_14"

    """
    START_TIME = dt.now().strftime("%Y_%m_%d__%H_%M")

    if args.custom_mlp:
        return "MLP_B_" + args.body + "_P_" + args.pi + "_V_" + args.vf + "_" + args.relu + "_" + START_TIME
    if args.load is None:
        return args.agent + "_" + START_TIME
    return args.load


def get_paths(agent_name: str, args):
    """ function to generate agent specific paths """
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')

    PATHS = {
        'model' : os.path.join(dir, 'agents', agent_name),
        'tb' : os.path.join(dir, 'training_logs', 'tensorboard', agent_name),
        'eval' : os.path.join(dir, 'training_logs', 'train_eval_log', agent_name),
        'robot_setting' : os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', robot + '.model.yaml'),
        'robot_as' : os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs', 'default_settings.yaml'),
    }

    if args.load is None:
        os.makedirs(PATHS.get('model'))
        write_hyperparameters_to_file(agent_name, PATHS, robot, gamma, n_steps, ent_coef, learning_rate, vf_coef,max_grad_norm, gae_lambda, batch_size, 
                                    n_epochs, clip_range, discrete_action_space)
    else:
        if not os.path.isfile(os.path.join(PATHS.get('model'), AGENT_NAME + ".zip")) and not os.path.isfile(os.path.join(PATHS.get('model'), "best_model.zip")):
            raise FileNotFoundError("Couldn't find model named %s.zip' or 'best_model.zip' in '%s'" % (AGENT_NAME, PATHS.get('model')))

    print_hyperparameters_from_file(agent_name, PATHS)

    if not os.path.exists(PATHS.get('eval')):
        os.makedirs(PATHS.get('eval'))
    if args.tb is True:
        if not os.path.exists(PATHS.get('tb')):
            os.makedirs(PATHS.get('tb'))
    else:
        PATHS['tb'] = None

    return PATHS


if __name__ == "__main__":
    args, _ = parse_training_args()

    rospy.init_node("train_node")

    # generate agent name and model specific paths
    AGENT_NAME = get_agent_name(args)
    print("________ STARTING TRAINING WITH:  %s ________\n" % AGENT_NAME)
    PATHS = get_paths(AGENT_NAME, args)

    if args.n is None:
        n_timesteps = 6000
    else:
        n_timesteps = args.n

    # instantiate gym environment
    n_envs = 1
    task = get_predefined_task()
    env = DummyVecEnv([lambda: FlatlandEnv(task, PATHS.get('robot_setting'), PATHS.get('robot_as'), discrete_action_space)] * n_envs)
   
    # instantiate eval environment
    eval_env = Monitor(FlatlandEnv(task, PATHS.get('robot_setting'), PATHS.get('robot_as'), discrete_action_space), PATHS.get('eval'))
    eval_env = EvalCallback(eval_env, n_eval_episodes=5, eval_freq=100, log_path=PATHS.get('eval'), best_model_save_path=PATHS.get('model'), deterministic=True)
                            

    # determine mode
    if args.custom_mlp:
        # custom mlp flag
        model = PPO("MlpPolicy", env, policy_kwargs = dict(net_arch = args.net_arch, activation_fn = get_act_fn(args.act_fn)), 
                    gamma = gamma, n_steps = n_steps, ent_coef = ent_coef, learning_rate = learning_rate, vf_coef = vf_coef, 
                    max_grad_norm = max_grad_norm, gae_lambda = gae_lambda, batch_size = batch_size, n_epochs = n_epochs, clip_range = clip_range, 
                    tensorboard_log = PATHS.get('tb'), verbose = 1)

    elif args.agent is not None:
        # predefined agent flag
        if args.agent == "MLP_ARENA2D":
                model = PPO(MLP_ARENA2D_POLICY, env, gamma = gamma, n_steps = n_steps, ent_coef = ent_coef, 
                        learning_rate = learning_rate, vf_coef = vf_coef, max_grad_norm = max_grad_norm, gae_lambda = gae_lambda, 
                        batch_size = batch_size, n_epochs = n_epochs, clip_range = clip_range, tensorboard_log = PATHS.get('tb'), verbose = 1)

        elif args.agent == "DRL_LOCAL_PLANNER" or args.agent == "CNN_NAVREP":
            if args.agent == "DRL_LOCAL_PLANNER":
                policy_kwargs = policy_kwargs_drl_local_planner
            else:
                policy_kwargs = policy_kwargs_navrep

            model = PPO("CnnPolicy", env, policy_kwargs = policy_kwargs, 
                gamma = gamma, n_steps = n_steps, ent_coef = ent_coef, learning_rate = learning_rate, vf_coef = vf_coef, 
                max_grad_norm = max_grad_norm, gae_lambda = gae_lambda, batch_size = batch_size, n_epochs = n_epochs, 
                clip_range = clip_range, tensorboard_log = PATHS.get('tb'), verbose = 1)
    
    else:
        # load flag
        if os.path.isfile(os.path.join(PATHS.get('model'), AGENT_NAME + ".zip")):
            model = PPO.load(os.path.join(PATHS.get('model'), AGENT_NAME), env)
        elif os.path.isfile(os.path.join(PATHS.get('model'), "best_model.zip")):
            model = PPO.load(os.path.join(PATHS.get('model'), "best_model"), env)

    # start training
    model.learn(total_timesteps = n_timesteps, callback=eval_env, reset_num_timesteps = False)
    #model.save(os.path.join(PATHS.get('model'), AGENT_NAME))

    # update the timesteps the model has trained in total
    update_total_timesteps_in_file(n_timesteps, PATHS)
    print("training done and model saved!")
    
"""
    s = time.time()
    model.learn(total_timesteps = 3000)
    print("steps per second: {}".format(1000 / (time.time() - s)))
    # obs = env.reset()
    # for i in range(1000):
    #     action, _state = model.predict(obs, deterministic = True)
    #     obs, reward, done, info = env.step(action)
    #     env.render()
    #     if done:
    #       obs = env.reset()
"""