import os
import rospy
import rospkg
from stable_baselines3 import PPO

from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.argsparser import parse_run_agent_args

if __name__ == "__main__":
    args, _ = parse_run_agent_args()

    rospy.init_node("run_node")

    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    PATHS={
        'model': os.path.join(dir, 'agents', args.load)
    }

    assert os.path.isfile(
        os.path.join(PATHS['model'], "best_model.zip")), "No model file found in %s" % PATHS['model']