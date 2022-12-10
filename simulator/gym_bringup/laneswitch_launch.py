"""
steps:
- run two agents on different racelines (see API) -> see render function
- write simple overtake statemachine
"""

import yaml
import gym
import numpy as np
from argparse import Namespace
from f110_gym.envs.base_classes import Integrator
from planners.pure_pursuit_planner import PurePursuitPlanner
import monitor_utils.bagging_utils as bagger
import time
from monitor_utils.transform_utils import FrenetFrame
# from scenarios.lane_switcher import LaneSwitcher

#look into updating rendering (camera following car/cars or full zoom out (and rendering))

def main():
    #load configuration yaml
    with open('configuration/config_parameters.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict) #configuration dictionary

    def render_callback(env_renderer):
        # custom extra drawing function

        e = env_renderer

        # update camera to follow car
        x = e.cars[0].vertices[::2]
        y = e.cars[0].vertices[1::2]
        top, bottom, left, right = max(y), min(y), min(x), max(x)
        e.score_label.x = left
        e.score_label.y = top - 700*2
        e.left = left - 800*2
        e.right = right + 800*2
        e.top = top + 800*2
        e.bottom = bottom - 800*2

        attacker_planner.render_waypoints(env_renderer)
        defender_planner.render_waypoints(env_renderer)

    #initialise environment
    env = gym.make('f110_gym:f110-v0', map=conf.map['map_path'], map_ext=conf.map['map_ext'], num_agents=conf.map['num_agents'], timestep=0.01, integrator=Integrator.RK4)
    # env.add_render_callback(render_callback)
    obs, step_reward, done, info = env.reset(np.array([[conf.agents['defender_sx'], conf.agents['defender_sy'], conf.agents['defender_stheta']], [conf.agents['attacker_sx'], conf.agents['attacker_sy'], conf.agents['attacker_stheta']]])) #fix this with the right instantiation

    # print(conf.agents['defender_sx'], conf.agents['defender_sy'])

    #setup defdender
    if(conf.agents['is_defender']):
        if (conf.agents['defender_planner'] == 'pure_pursuit'):
            defender_waypoints = bagger.csvtonp(conf.agents['defender_wpt_path'])
            defender_la = conf.agents['defender_lookahead_distance']
            defender_vgain = conf.agents['defender_vgain']
            defender_planner = PurePursuitPlanner(defender_waypoints, defender_la, defender_vgain, (0.17145+0.15875))
    else:
        raise Exception("No Defender instantiated. Terminating... ")

    #setup attacker
    if(conf.agents['is_attacker']):
        if(conf.agents['planner_attacker'] == 'pure_pursuit'):
            attacker_waypoints = bagger.csvtonp(conf.agents['attacker_wpt_path'])
            attacker_la = conf.agents['attacker_lookahead_distance']
            attacker_vgain = conf.agents['attacker_vgain']
            attacker_planner = PurePursuitPlanner(attacker_waypoints, attacker_la, attacker_vgain, (0.17145+0.15875))
    else:
        raise Exception("No Attacker instantiated. Terminating... ")
    
    # instantiating the environment
    env.render()

    # simulation loop
    lap_time = 0.0
    start=time.time()
    counter = 0

    cl = bagger.csvtonp('~/STL_workspace/simulator/maps/centerline.csv')
    frenet1 = FrenetFrame(cl, data = 'raw')
    # scenario_planner = LaneSwitcher(frenet1)

    # loops when env not done
    while not done:

        throttle_d, steer_d = defender_planner.plan(heading = np.array([obs['poses_theta'][0]]), position=np.array([[obs['poses_x'][0], obs['poses_y'][0]]]))
        throttle_a, steer_a = attacker_planner.plan(heading = np.array([obs['poses_theta'][1]]), position=np.array([[obs['poses_x'][1], obs['poses_y'][1]]]))
        # stepping through the environment
        obs, step_reward, done, info = env.step(np.array([[steer_d, throttle_d],[steer_a, throttle_a]]))
        
        lap_time += step_reward

        # print(frenet1.gap(obs[]))

        env.render(mode='human_fast')
        # counter = counter+1
        # if counter == 100:
        #     time.sleep(1000)
        #     print ('Sleeping...')

if __name__ == '__main__':
    main()