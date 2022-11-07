import yaml
from argparse import Namespace
import gym
import numpy as np
import planners.pure_pursuit_planner as PurePursuitPlanner

#from  import planner # the policy/motion planner that you create

def main():
    #load configuration yaml
    with open('config_map.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict) #configuration dictionary

    #load waypoints as a list
    waypoints = np.loadtext(conf.wpt_path, delimiter=conf.wpt_delim)

    print(waypoints)
    # instantiating the environment
    env = gym.make('f110_gym:f110-v0')
    obs, step_reward, done, info = env.reset(np.array([[0., 0., 0.], [2., 0., 0.]])) # pose of both agents 

    # instantiating your policy
    planner = planner()

    # simulation loop
    lap_time = 0.

    # loops when env not done
    while not done:
        # get action based on the observation
        #actions = planner.plan(obs)

        # stepping through the environment
        obs, step_reward, done, info = env.step(actions)

        lap_time += step_reward

if __name__ == '__main__':
    main()