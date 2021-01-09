import rospy

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
"""
from tools.argsparser import parse_training_args
from task_generator.tasks import get_predefined_task
from scripts.custom_policy import *
"""
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.argsparser import parse_training_args, get_act_fn
from task_generator.task_generator.tasks import get_predefined_task
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.scripts.custom_policy import *
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.envs.flatland_gym_env import FlatlandEnv

###HYPERPARAMETER###
gamma = 0.99
n_steps = 128
ent_coef = 0.01
learning_rate = 2.5e-4
vf_coef = 0.5
max_grad_norm = 0.5
lam = 0.95
nminibatches = 64
noptepochs = 4
cliprange = 0.2
####################


if __name__ == "__main__":
    args, _ = parse_training_args()

    if args.n is None:
        n_timesteps = 60000
    else:
        n_timesteps = args.n
    
    rospy.init_node("train_node")
    task = get_predefined_task()

    ROBOT_SETTING_PATH = rospkg.RosPack().get_path('simulator_setup')
    ROBOT_AS_PATH = rospkg.RosPack().get_path('arena_local_planner_drl')
    yaml_ROBOT_SETTING_PATH = os.path.join(ROBOT_SETTING_PATH, 'robot', 'myrobot.model.yaml')
    yaml_ROBOT_AS_PATH = os.path.join(ROBOT_AS_PATH, 'configs', 'default_settings.yaml')
    
    n_envs = 1
    env = DummyVecEnv([lambda: FlatlandEnv(task, yaml_ROBOT_SETTING_PATH, yaml_ROBOT_AS_PATH, True)] * n_envs)

    # check which mode
    if args.custom_mlp:
        print(args.net_arch)
        print(args.act_fn)
        # check if model already exists, load
        model = PPO("MlpPolicy", env, policy_kwargs=dict(net_arch=args.net_arch, activation_fn=get_act_fn(args.act_fn)),
                    verbose=0, gamma=gamma, n_steps=n_steps, ent_coef=ent_coef,
                    learning_rate=learning_rate, vf_coef=vf_coef, max_grad_norm=max_grad_norm, gae_lambda=lam,
                    batch_size=nminibatches, n_epochs=noptepochs, clip_range=cliprange)
    else:
        # check if model already exists, load
        
        if args.agent == "MLP_ARENA2D":
            model = PPO(MLP_ARENA2D_POLICY, env, verbose=0, gamma=gamma, n_steps=n_steps, ent_coef=ent_coef,
                    learning_rate=learning_rate, vf_coef=vf_coef, max_grad_norm=max_grad_norm, gae_lambda=lam,
                    batch_size=nminibatches, n_epochs=noptepochs, clip_range=cliprange)

        elif args.agent == "DRL_LOCAL_PLANNER" or args.agent == "CNN_NAVREP":

            if args.agent == "DRL_LOCAL_PLANNER":
                policy_kwargs = policy_kwargs_drl_local_planner
            else:
                policy_kwargs = policy_kwargs_navrep

            model = PPO("CnnPolicy", env, policy_kwargs=policy_kwargs,
                gamma=gamma, n_steps=n_steps, ent_coef=ent_coef, learning_rate=learning_rate, vf_coef=vf_coef,
                max_grad_norm=max_grad_norm, gae_lambda=lam, batch_size=nminibatches, n_epochs=noptepochs,
                clip_range=cliprange, verbose=1)

    model.learn(total_timesteps=n_timesteps)


"""
    n_envs = 1
    env = DummyVecEnv([lambda: FlatlandEnv(task, yaml_ROBOT_SETTING_PATH, yaml_ROBOT_AS_PATH, True)] * n_envs)

    #policy_kwargs_navrep['features_extractor_kwargs']['laser_num_beams'] = get_laser_num_beams(yaml_ROBOT_SETTING_PATH)
    model = PPO("CnnPolicy", env, policy_kwargs=policy_kwargs_drl_local_planner,
                gamma=gamma, n_steps=n_steps, ent_coef=ent_coef, learning_rate=learning_rate, vf_coef=vf_coef,
                max_grad_norm=max_grad_norm, gae_lambda=lam, batch_size=nminibatches, n_epochs=noptepochs,
                clip_range=cliprange, verbose=1)
    import time

    s = time.time()
    model.learn(total_timesteps=3000)
    print("steps per second: {}".format(1000 / (time.time() - s)))
    # obs = env.reset()
    # for i in range(1000):
    #     action, _state = model.predict(obs, deterministic=True)
    #     obs, reward, done, info = env.step(action)
    #     env.render()
    #     if done:
    #       obs = env.reset()
"""