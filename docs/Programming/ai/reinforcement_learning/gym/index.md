---
title: Using RL with Gymnasium
tags:
    - rl
    - gym
    - reinforcement
    - gymnasium
---


{{ page_folder_links() }}

<div class="grid-container">
    <div class="grid-item">
        <a href="custom_environment">
        <p>Custom Env</p></a>
    </div>
</div>

## Gymnasium main concepts
- **Observation Space**: Set of possible state that agent can observe in the environment
- **Action Space**: Set of actions that agent cant take in environment
- **Episode**: A complete run through the environment from initial state until terminate state is reach, each episode is composed if a sequence of states, actions and rewards 
- **Wrapper**: A tool in GYM that allow modify an environment behavior without changing its code, for example and time constrain and action masking
- Benchmark: Help to compare between different RL algorithm



## Observation space
What information the environment gives the agent


## Action space
What actions the agent is allowed to take

### 
- next_state: the observation after taking the action
- reward: the reward after tacking the action
- terminated: boolean, true if episode ended
- truncated: boolean , true if the episode end by early truncation (time limit reached)
- info: a dictionary contain additional environment information (for example in atari game it's hold user lives)

---

## Reference
- [gym solution](https://github.com/johnnycode8/gym_solutions)
- [Build a Custom Gymnasium Reinforcement Learning Environment & Train w Q-Learning & Stable Baselines3](https://youtu.be/AoGRjPt-vms)
- [gym turtlebot](https://github.com/anurye/gym-turtlebot)
