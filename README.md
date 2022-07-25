# Deep Reinforcement Learning (DRL) Algorithms with PyTorch

This repository contains PyTorch implementations of deep reinforcement learning algorithms. **The repository will soon be updated including the PyBullet environments!**

## Algorithms Implemented

1. Deep Q-Network (DQN) <sub><sup> ([V. Mnih et al. 2015](https://storage.googleapis.com/deepmind-media/dqn/DQNNaturePaper.pdf)) </sup></sub>
2. Double DQN (DDQN) <sub><sup> ([H. Van Hasselt et al. 2015](https://arxiv.org/abs/1509.06461)) </sup></sub>
3. Advantage Actor Critic (A2C)
4. Vanilla Policy Gradient (VPG)
5. Natural Policy Gradient (NPG) <sub><sup> ([S. Kakade et al. 2002](http://papers.nips.cc/paper/2073-a-natural-policy-gradient.pdf)) </sup></sub>
6. Trust Region Policy Optimization (TRPO) <sub><sup> ([J. Schulman et al. 2015](https://arxiv.org/abs/1502.05477)) </sup></sub>
7. Proximal Policy Optimization (PPO) <sub><sup> ([J. Schulman et al. 2017](https://arxiv.org/abs/1707.06347)) </sup></sub>
8. Deep Deterministic Policy Gradient (DDPG) <sub><sup> ([T. Lillicrap et al. 2015](https://arxiv.org/abs/1509.02971)) </sup></sub>
9. Twin Delayed DDPG (TD3) <sub><sup> ([S. Fujimoto et al. 2018](https://arxiv.org/abs/1802.09477)) </sup></sub>
10. Soft Actor-Critic (SAC) <sub><sup> ([T. Haarnoja et al. 2018](https://arxiv.org/abs/1801.01290)) </sup></sub>
11. SAC with automatic entropy adjustment (SAC-AEA) <sub><sup> ([T. Haarnoja et al. 2018](https://arxiv.org/abs/1812.05905)) </sup></sub>

## Environments Implemented

1. Classic control environments (CartPole-v1, Pendulum-v0, etc.) <sub><sup> (as described in [here](https://gym.openai.com/envs/#classic_control)) </sup></sub>
2. MuJoCo environments (Hopper-v2, HalfCheetah-v2, Ant-v2, Humanoid-v2, etc.) <sub><sup> (as described in [here](https://gym.openai.com/envs/#mujoco)) </sup></sub>
3. **PyBullet environments (HopperBulletEnv-v0, HalfCheetahBulletEnv-v0, AntBulletEnv-v0, HumanoidDeepMimicWalkBulletEnv-v1 etc.)** <sub><sup> (as described in [here](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_envs)) </sup></sub>

## Results (MuJoCo, PyBullet)

### MuJoCo environments

#### Hopper-v2

- Observation space: 8
- Action space: 3

#### HalfCheetah-v2

- Observation space: 17
- Action space: 6

#### Ant-v2

- Observation space: 111
- Action space: 8

#### Humanoid-v2

- Observation space: 376
- Action space: 17

### PyBullet environments

#### HopperBulletEnv-v0

- Observation space: 15
- Action space: 3

#### HalfCheetahBulletEnv-v0

- Observation space: 26
- Action space: 6

#### AntBulletEnv-v0

- Observation space: 28
- Action space: 8

#### HumanoidDeepMimicWalkBulletEnv-v1

- Observation space: 197
- Action space: 36

## Requirements

- [PyTorch](https://pytorch.org)
- [TensorBoard](https://pytorch.org/docs/stable/tensorboard.html)
- [gym](https://github.com/openai/gym)
- [mujoco-py](https://github.com/openai/mujoco-py)
- [PyBullet](https://pybullet.org/wordpress/)


