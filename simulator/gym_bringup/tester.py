import yaml
import gym
import numpy as np
from argparse import Namespace
from f110_gym.envs.base_classes import Integrator
from planners.pure_pursuit_planner import PurePursuitPlanner
import time

#look into updating rendering (camera following car/cars or full zoom out (and rendering))

#def derive_og (scans, conf):
    

def main():
    #load configuration yaml
    with open('configuration/config_parameters.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict) #configuration dictionary

    #setup defdender
    if(conf.agents['is_defender'] == 'true'):
        if (conf.agents['defender_planner'] == 'pure_pursuit'):
            defender_waypoints = np.loadtxt(conf.agents['defender_wpt_path'], delimiter=conf.agents['defender_wpt_delim'])
            defender_la = conf.agents['defender_lookahead_distance']
            defender_vgain = conf.agents['defender_vgain']
            defender_planner = PurePursuitPlanner(defender_waypoints, defender_la, defender_vgain)
    else:
        raise Exception("No Defender instantiated. Terminating... ")
    
    #setup attacker
    if(conf.agents['is_attacker'] == 'false'):
        if(conf.agents['planner_attacker'] == 'rrt'):
            pass
    else:
        pass
    
    # instantiating the environment
    env = gym.make('f110_gym:f110-v0', map=conf.map['map_path'], map_ext=conf.map['map_ext'], num_agents=conf.map['num_agents'], timestep=0.01, integrator=Integrator.RK4)
    obs, step_reward, done, info = env.reset(np.array([[conf.agents['defender_sx'], conf.agents['defender_sy'], conf.agents['defender_stheta']]])) 
    env.render()

    # simulation loop
    laptime = 0.0

    # loops when env not done
    while not done:
        obs, step_reward, done, info = env.step(np.array([[0.0, 0.0]]))
        # derive_og (obs['scans'])
        print(obs['scans'])
        time.sleep(1000)
        laptime += step_reward
        env.render(mode='human')

if __name__ == '__main__':
    main()