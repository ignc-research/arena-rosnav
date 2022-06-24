from arena_marl.marl_agent.utils.supersuit_utils import MarkovVectorEnv_patched


def store_data():
    """
    Store data in a csv file
    """


class Env_Recorder(MarkovVectorEnv_patched):
    """
    A wrapper for the environment that records the state and action
    """

    def __init__(self):
        super(Env_Recorder, self).step(self, actions)
