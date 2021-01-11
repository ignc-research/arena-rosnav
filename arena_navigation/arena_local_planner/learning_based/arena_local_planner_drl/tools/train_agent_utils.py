import os
import datetime


def write_hyperparameters_to_file(agent_name: str, PATHS: dict, robot, gamma, n_steps, ent_coef, learning_rate, vf_coef, max_grad_norm, gae_lambda, batch_size, n_epochs, clip_range):
    """ function to document hyperparameters in the model's directory """
    doc_location = os.path.join(PATHS.get('model'), "hyperparameters.txt")

    with open(doc_location, "w+") as f:
        f.write("'%s'\n" % agent_name)
        f.write("_____________________________________\n")
        f.write("robot = %s \n" % robot)
        f.write("gamma = %f \n" % gamma)
        f.write("n_steps = %d \n" % n_steps)
        f.write("ent_coef = %f \n" % ent_coef)
        f.write("learning_rate = %f \n" % learning_rate)
        f.write("vf_coef = %f \n" % vf_coef)
        f.write("max_grad_norm = %f \n" % max_grad_norm)
        f.write("gae_lambda = %f \n" % gae_lambda)
        f.write("batch_size = %d \n" % batch_size)
        f.write("n_epochs = %d \n" % n_epochs)
        f.write("clip_range = %f \n" % clip_range)
        # total_timesteps has to be in the last line!
        f.write("total_timesteps = %d" % 0)


def update_total_timesteps_in_file(timesteps: int, PATHS: dict):
    """ function to update number of total timesteps in the file containing the hyperparameters """
    doc_location = os.path.join(PATHS.get('model'), "hyperparameters.txt")

    if os.path.isfile(doc_location):
        with open(doc_location, "r") as file:
            parameters = file.readlines()
        
        # extracts number from the last line and converts it to int
        # total_timesteps has to be in the last line!
        if "total_timesteps" in parameters[len(parameters)-1]:
            current_total_timesteps = int(''.join(list(filter(str.isdigit, parameters[len(parameters)-1]))))
            current_total_timesteps += timesteps

            parameters = parameters[:-1]
            parameters.append("total_timesteps = %d" % current_total_timesteps)

            with open(doc_location, "w") as file:
                file.write("".join(parameters))
        else:
            print("Parameter 'total_timesteps' not in last line of 'hyperparameter.txt!'")
    else:
        print("Found no 'hyperparameters.txt' in model directory (%s) !" % PATHS.get('model'))


def print_hyperparameters_from_file(agent_name: str, PATHS: dict):
    """ function to print hyperparameters from agent-specific hyperparameter file """
    doc_location = os.path.join(PATHS.get('model'), "hyperparameters.txt")

    if os.path.isfile(doc_location):
        with open(doc_location, "r") as file:
            parameters = file.readlines()
        
        print("\n--------------------------------")
        print("         HYPERPARAMETERS         \n")
        for line in parameters[2:]:
            print(line)
        print("--------------------------------\n\n")
