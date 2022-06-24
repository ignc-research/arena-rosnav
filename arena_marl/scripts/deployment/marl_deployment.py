from marl_agent.utils.utils import (
    instantiate_deploy_drl_agents,
    get_hyperparameter_file,
)

from marl_tools.train_agent_utils import (
    load_config,
)
import rospy

from tools.argsparser import parse_training_args


def main(args):
    # load configuration
    config = load_config(args.config)

    # set debug_mode
    rospy.set_param("debug_mode", config["debug_mode"])

    # load agents
    assert len(config["robots"]) == 1, "Only one robot is supported"

    robots = {}
    # loop over all robots in config
    for robot_name, robot_train_params in config["robots"].items():
        robots[robot_name] = instantiate_deploy_drl_agents(
            num_robots=robot_train_params["num_robots"],
            robot_model=robot_name,
            hyperparameter_path=get_hyperparameter_file(
                robot_train_params["hyperparameter_path"]
            ),
        )

    episode_rewards, episode_lengths = [], []

    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    args, _ = parse_training_args()
    main(args)
