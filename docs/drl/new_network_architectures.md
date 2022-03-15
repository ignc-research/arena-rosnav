# Implementation of new Network Architectures

Stable Baselines3 provides policy networks for images (CnnPolicies), other type of input features (MlpPolicies) and multiple different inputs (MultiInputPolicies).

### SB3 networks are separated into two mains parts

- A features extractor (usually shared between actor and critic when applicable, to save computation) whose role is to extract features (i.e. convert to a feature vector) from high-dimensional observations, for instance, a CNN that extracts features from images. This is the _features_extractor_class parameter_. You can change the default parameters of that features extractor by passing a _features_extractor_kwargs_ parameter.

- A (fully-connected) network that maps the features to actions/value. Its architecture is controlled by the net_arch parameter.

Usually, each SB3 network has a features extractor followed by a fully-connected network.

### There are two recommended ways to implement new custom networks

1. The easier method is to inheret from the [BaseAgent class](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/rl_agent/model/base_agent.py) which essentially provides an interface that generates policy kwargs for desired model architecture for the PPO initialization (as is described in [the official documentation](https://stable-baselines3.readthedocs.io/en/master/guide/custom_policy.html#on-policy-algorithms)). One can simply use the `net_arch` parameter to build up a custom multilayer perceptron feature extractors. Furthermore, one can implement custom CNN feature extractors with the framework **torch**. For examples, please refer to [features_extractors.py](https://github.com/ignc-research/arena-rosnav/blob/436861bda41d6a7af5c644d4e062d47a6b97e2b1/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/rl_agent/model/feature_extractors.py) and [custom_sb3_policy.py](https://github.com/ignc-research/arena-rosnav/blob/436861bda41d6a7af5c644d4e062d47a6b97e2b1/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/rl_agent/model/custom_sb3_policy.py).

2. The more advanced method is to write and define the policy by oneself. This can be advantageous, when your task requires even more granular control over the policy/value architecture. Examples can be found in [the official documentation](https://stable-baselines3.readthedocs.io/en/master/guide/custom_policy.html#advanced-example) and also in [custom_policy.py](https://github.com/ignc-research/arena-rosnav/blob/436861bda41d6a7af5c644d4e062d47a6b97e2b1/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/rl_agent/model/custom_policy.py).

### What to do with the model after implementation

For the purpose of dynamic organization of the model architectures in a central entity, we implemented a policy registry that can be extended by simply adding a decorator to the architectures' class.

In order to add the policy to the registry, one needs to call the _register()_-method of the [`AgentFactory`](https://github.com/ignc-research/arena-rosnav/blob/436861bda41d6a7af5c644d4e062d47a6b97e2b1/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/rl_agent/model/agent_factory.py) with an unique identifier for the architecture:

```python
@AgentFactory.register("AGENT_23")
class AGENT_23(BaseAgent):
    type = PolicyType.CNN
    features_extractor_class = EXTRACTOR_5
    features_extractor_kwargs = dict(features_dim=128)
    net_arch = [128, dict(pi=[64, 64, 64], vf=[64, 64, 64])]
    activation_fn = nn.ReLU
```

Afterwards, the model can be trained with the training script by specifying the `--agent` parameter with its respective identifier.

**Note**:
Scripts must import the following modules in order to register the policies:

```python
import rl_agent.model.custom_policy
import rl_agent.model.custom_sb3_policy
```
