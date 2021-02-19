import torch as th
import argparse


def get_net_arch(args: argparse.Namespace):
    """ function to convert input args into valid syntax for the PPO """
    body = parse_string(args.body)
    policy = parse_string(args.pi)
    value = parse_string(args.vf)
    return body + [dict(vf=value, pi=policy)]


def parse_string(string: str):
    """ function to convert a string into a int list 
    
    Example:

    Input: parse_string("64-64") 
    Output: [64, 64]

    """
    string_arr = string.split("-")
    int_list = []
    for string in string_arr:
        try:
            int_list.append(int(string))
        except:
            raise Exception("Invalid argument format on: " + string)
    return int_list


def get_act_fn(act_fn_string: str):
    """ function to convert str into pytorch activation function class """
    if act_fn_string == "relu":
        return th.nn.ReLU
    elif act_fn_string == "sigmoid":
        return th.nn.Sigmoid
    elif act_fn_string == "tanh":
        return th.nn.Tanh