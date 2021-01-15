import argparse
import os

from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.custom_mlp_args_utils import *



def training_args(parser):
    """ program arguments training script """
    parser.add_argument('--no-gpu', action='store_true', help='disables gpu for training')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--agent', type=str,
                        choices=['MLP_ARENA2D', 'DRL_LOCAL_PLANNER', 'CNN_NAVREP'],
                        help='predefined agent to train')
    group.add_argument('--custom-mlp', action='store_true', help='enables training with custom multilayer perceptron')
    group.add_argument('--load', type=str, metavar="[agent name]", help='agent to be loaded for training')
    parser.add_argument('--n', type=int, help='timesteps in total to be generated for training')

    parser.add_argument('-log', '--eval_log', action='store_true', help='enables storage of evaluation data')

    parser.add_argument('--tb', action='store_true', help='enables tensorboard logging')


def custom_mlp_args(parser):
    """ arguments for the custom mlp mode """
    custom_mlp_args = parser.add_argument_group('custom mlp args', 'architecture arguments for the custom mlp')

    custom_mlp_args.add_argument('--body', type=str, default="", metavar="{num}-{num}-...",
                                help="architecture of the shared latent network, "
                                "each number representing the number of neurons per layer")
    custom_mlp_args.add_argument('--pi', type=str, default="", metavar="{num}-{num}-...",
                                help="architecture of the latent policy network, "
                                "each number representing the number of neurons per layer")
    custom_mlp_args.add_argument('--vf', type=str, default="", metavar="{num}-{num}-...",

                                help="architecture of the latent value network, "
                                "each number representing the number of neurons per layer")
    custom_mlp_args.add_argument('--act_fn', type=str, default="relu", choices=['relu', 'sigmoid', 'tanh'],
                                help="activation function to be applied after each hidden layer")


def process_training_args(parsed_args):
    """ argument check function """
    if parsed_args.no_gpu:
        os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
    if parsed_args.custom_mlp:
        setattr(parsed_args, 'net_arch', get_net_arch(parsed_args))
        delattr(parsed_args, 'agent')

        delattr(parsed_args, 'load')
    else:

        if parsed_args.body is not "" or parsed_args.pi is not "" or parsed_args.vf is not "":
            print("[custom mlp] arguments will be ignored..")
        delattr(parsed_args, 'body')
        delattr(parsed_args, 'pi')
        delattr(parsed_args, 'vf')
        delattr(parsed_args, 'act_fn')


def parse_training_args(args=None, ignore_unknown=False):
    """ parser for training script """
    arg_populate_funcs = [training_args, custom_mlp_args]
    arg_check_funcs = [process_training_args]

    return parse_various_args(args, arg_populate_funcs, arg_check_funcs, ignore_unknown)


def parse_various_args(args, arg_populate_funcs, arg_check_funcs, ignore_unknown):
    """ generic arg parsing function """
    parser = argparse.ArgumentParser()

    for func in arg_populate_funcs:
        func(parser)

    if ignore_unknown:
        parsed_args, unknown_args = parser.parse_known_args(args=args)
    else:
        parsed_args = parser.parse_args(args=args)
        unknown_args = []

    for func in arg_check_funcs:
        func(parsed_args)

    print_args(parsed_args)
    return parsed_args, unknown_args


def print_args(args):
    print("\n-------------------------------")
    print("            ARGUMENTS          ")
    for k in args.__dict__:
        print("- {} : {}".format(k, args.__dict__[k]))
    print("--------------------------------\n")

