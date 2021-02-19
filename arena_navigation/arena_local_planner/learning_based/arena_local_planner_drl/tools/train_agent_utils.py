import os
import datetime
import json

def write_hyperparameters_json(hyperparams: object, PATHS: dict):
    doc_location = os.path.join(PATHS.get('model'), "hyperparameters.json")

    with open(doc_location, "w", encoding='utf-8') as target:
        json.dump(hyperparams.__dict__, target, ensure_ascii=False, indent=4)


def update_total_timesteps_json(timesteps: int, PATHS:dict):
    doc_location = os.path.join(PATHS.get('model'), "hyperparameters.json")

    if os.path.isfile(doc_location):
        with open(doc_location, "r") as file:
            hyperparams = json.load(file)
        try:
            curr_timesteps = int(hyperparams['n_timesteps']) + timesteps
            hyperparams['n_timesteps'] = curr_timesteps
        except Exception:
            print("Parameter 'total_timesteps' not found or not of type Integer in 'hyperparameter.json'!")
        else:
            with open(doc_location, "w", encoding='utf-8') as target:
                json.dump(hyperparams, target, ensure_ascii=False, indent=4)
    else:
        print("Found no 'hyperparameters.json' in model directory (%s) !" % PATHS.get('model'))


def print_hyperparameters_json(hyperparams_obj: object, PATHS:dict):
    doc_location = os.path.join(PATHS.get('model'), "hyperparameters.json")

    if os.path.isfile(doc_location):
        with open(doc_location, "r") as file:
            hyperparams = json.load(file)

        if not check_hyperparam_format(hyperparams_obj, hyperparams):
            raise AssertionError("'hyperparameters.json' in %s has wrong format!" % PATHS.get('model'))

        print("\n--------------------------------")
        print("         HYPERPARAMETERS         \n")
        for param, param_val in hyperparams.items():
            print(f"{param}:    {param_val}")
        print("--------------------------------\n\n")
    else:
        print("Warning: found no 'hyperparameter.json' in %s" % PATHS.get('model'))


def check_hyperparam_format(hyperparams_obj: object, loaded_hyperparams: dict):
    if set(hyperparams_obj.__dict__.keys()) == set(loaded_hyperparams.keys()):
        return True
    return False
