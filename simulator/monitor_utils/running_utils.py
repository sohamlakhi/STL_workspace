import time
import yaml
from argparse import Namespace

#NOTE: consider using decorators to do this

#run on module import
with open('configuration/config_parameters.yaml') as file:
    conf_dict = yaml.load(file, Loader=yaml.FullLoader)
conf = Namespace(**conf_dict) #configuration dictionary

# to slow down the plot enough to see and debug
def plot_pause():
    if conf.time['plot_pause']:
        input('Please press enter to continue to the next step')
        return
    else:
        return

def plot_delay(t = 1):
    if conf.time['plot_sleep']:
        time.sleep(t)
        return
    else:
        return

# simulate pause to debug code
def debug_delay(t = 1000):
    if conf.time['debug_sleep']:
        print('Paused for ' + str(t) + ' seconds')
        time.sleep(t)
        return
    else:
        return

def debug_pause():
    if conf.time['debug_pause']:
        input('Please press enter to continue to the next step')
        return
    else:
        return

def debug_print(item):
    if conf.time['debug_print']:
        input('Please press enter to continue to the next step')
        return
    else:
        return
