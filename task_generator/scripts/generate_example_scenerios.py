from task_generator.tasks import ScenerioTask


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser("Generate a example json file which defines several scenerios")
    parser.add_argument('--dst_json_path')
    args = parser.parse_args()
    ScenerioTask.generate_scenerios_json_example(args.dst_json_path)



