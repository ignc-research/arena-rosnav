from abc import abstractmethod

import numpy as np


class ModelBase:
    """Abtract base class, all models have to implement get_next_action method."""

    def __init__(self,
                 observation_info: dict, name: str):
        self._observation_info = observation_info
        self._name = name

    @abstractmethod
    def get_next_action(self, observation_dict: dict) -> np.ndarray:
        pass

    @abstractmethod
    def wait_for_agent(self) -> bool:
        pass

    @abstractmethod
    def reset(self):
        pass

    def close(self):
        pass

    def get_observation_info(self) -> dict:
        return self._observation_info

    def get_name(self) -> str:
        return self._name
