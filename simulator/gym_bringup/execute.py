import yaml
import gym
import numpy as np
from argparse import Namespace
from f110_gym.envs.base_classes import Integrator
from planners.pure_pursuit_planner import PurePursuitPlanner
import monitor_utils.bagging_utils as bagger
import time

#look into updating rendering (camera following car/cars or full zoom out (and rendering))

def main():
    #load configuration yaml
    with open('configuration/config_parameters_single.yaml') as file:
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
        e.score_label.y = top - 700
        e.left = left - 800
        e.right = right + 800
        e.top = top + 800
        e.bottom = bottom - 800

        defender_planner.render_waypoints(env_renderer)

    #initialise environment
    env = gym.make('f110_gym:f110-v0', map=conf.map['map_path'], map_ext=conf.map['map_ext'], num_agents=conf.map['num_agents'], timestep=0.01, integrator=Integrator.RK4)
    env.add_render_callback(render_callback)
    obs, step_reward, done, info = env.reset(np.array([[conf.agents['defender_sx'], conf.agents['defender_sy'], conf.agents['defender_stheta']]])) #fix this with the right instantiation

    print(conf.agents['defender_sx'], conf.agents['defender_sy'])

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
    if(conf.agents['is_attacker'] == 'false'):
        if(conf.agents['planner_attacker'] == 'rrt'):
            pass
    else:
        pass
    
    # instantiating the environment
    env.render()

    # simulation loop
    lap_time = 0.0
    start=time.time()
    counter = 0

    # loops when env not done
    while not done:
        throttle, steer = defender_planner.plan(heading = np.array([obs['poses_theta'][0]]), position=np.array([[obs['poses_x'][0], obs['poses_y'][0]]]))
        # stepping through the environment
        obs, step_reward, done, info = env.step(np.array([[steer, throttle]]))
        
        lap_time += step_reward

        env.render(mode='human_fast')
        # counter = counter+1
        # if counter == 100:
        #     time.sleep(1000)
        #     print ('Sleeping...')

if __name__ == '__main__':
    main()