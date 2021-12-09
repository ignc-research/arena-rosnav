from task_generator.tasks import ScenarioTask


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser("Generate a example json file which defines several scenarios")
    parser.add_argument('--dst_json_path')
    args = parser.parse_args()
    ScenarioTask.generate_scenarios_json_example(args.dst_json_path)



