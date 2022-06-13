from typing import Any, Union
import rospy
import rospkg
import yaml
import os
from std_msgs.msg import Bool
import subprocess
import time
import signal
import traceback
import argparse

__all__ = ["main", "evaluate_scenario"]


class EvalDeclerationFile:
    def __init__(
        self,
        local_planner: str,
        additional_args: Union[
            Any, None
        ],  # None or a List with dicts containing "value": str, "name": str
        scenarios: Any,  # Array of Dict with entries "robot": str and map_files: Array of Dict with "map_file": str, "scenario": str
        docker: Union[None, str],
    ) -> None:
        self.local_planner = local_planner
        self.arguments = additional_args
        self.scenarios = scenarios
        self.docker = docker

    def get_startup_command_creator(
        self, use_recorder: bool, use_rviz: bool, verbose: bool
    ) -> str:
        return (
            lambda robot, map_file, scenario: f"roslaunch arena_bringup start_arena_flatland.launch use_recorder:={use_recorder} gui:=false "
            f"use_rviz:={use_rviz} {self.build_arguments_string()} model:={robot} "
            f"scenario_file:={scenario} map_file:={map_file} local_planner:={self.local_planner}"
            + (f" > eval_{self.local_planner}.txt" if verbose else "")
        )

    def build_arguments_string(self) -> str:
        if not self.arguments:
            return ""

        return " ".join(
            [f"{arg['name']}:={arg['value']}" for arg in self.arguments]
        )

    def build_docker_command(self) -> Union[str, None]:
        if not self.docker:
            return None

        rosnav_2d_path = os.path.join(rospkg.RosPack().get_path("eval"), "..")
        return f"docker run -ti -v {rosnav_2d_path}/{self.docker['localPath']}:/{self.docker['dockerPath']} --network host {self.docker['name']}"

    @staticmethod
    def read_eval_file(file_name: str):
        file_path = os.path.join(
            rospkg.RosPack().get_path("eval"), "src", f"{file_name}.yaml"
        )
        print(file_path)
        def try_read_key(file, key):
            try:
                return file[key]
            except:
                return None

        with open(file_path, "r") as file:
            evals = yaml.safe_load(file)

            additional_arguments = try_read_key(evals, "additionalArguments")
            docker_name = try_read_key(evals, "docker")

            return EvalDeclerationFile(
                evals["local_planner"],
                additional_arguments,
                evals["scenarios"],
                docker_name,
            )


def evaluate_scenario(
    startup_command: str, docker_command: Union[str, None]
) -> None:
    """_summary_

    Args:
        startup_command (str): _description_
        docker_command (Union[str, None]): _description_
    """

    process = subprocess.Popen(
        startup_command, shell=True, preexec_fn=os.setsid
    )

    if docker_command:
        time.sleep(5)
        print("Launching docker")
        docker_process = subprocess.Popen(
            docker_command, shell=True, preexec_fn=os.setsid
        )

    time.sleep(1)

    rospy.init_node("run_evals_node", anonymous=True, disable_signals=True)
    rospy.wait_for_message("/End_of_scenario", Bool)

    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    time.sleep(1)
    os.system("killall roslaunch")

    process.wait(30)

    if docker_command:
        os.killpg(os.getpgid(docker_process.pid), signal.SIGINT)
        time.sleep(1)
        os.system("killall docker")

        docker_process.wait(10)

    time.sleep(10)


def main(args):
    use_recorder = args.use_recorder
    eval_file = args.eval_file
    verbose = args.verbose
    use_rviz = args.use_rviz

    eval_decleration_file = EvalDeclerationFile.read_eval_file(eval_file)

    if not eval_decleration_file:
        raise Exception()

    docker_command = eval_decleration_file.build_docker_command()
    build_startup_command = eval_decleration_file.get_startup_command_creator(
        use_recorder, use_rviz, verbose
    )

    for scenario in eval_decleration_file.scenarios:
        robot = scenario["robot"]
        map_files = scenario["map_files"]

        for map_file in map_files:
            current_map_file = map_file["map_file"]
            scenario_file = map_file["scenario"]

            startup_command = build_startup_command(
                robot, current_map_file, scenario_file
            )

            print(
                "==============================================================="
            )
            print(
                f"[{robot}] Starting scenario {scenario_file} in map_file {current_map_file}"
            )

            print(startup_command)

            evaluate_scenario(startup_command, docker_command)

            print(
                "==============================================================="
            )
            print(
                f"[{robot}] Scenario {scenario_file} in map_file {current_map_file} finished"
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-f", "--eval_file")
    parser.add_argument("-r", "--use_recorder", action="store_true")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--use_rviz", action="store_true")

    try:
        main(parser.parse_args())
    except:
        traceback.print_exc()
        os.system("killall roslaunch")
