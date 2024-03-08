from src.blimp_agent import BlimpAgent

import numpy as np
import time

bb=BlimpAgent()
bb.spawnAgent((0,0,1))
bb.sim.startSimulation()
ACTIVE=True

vec=np.zeros(3)


up=False
down=False
right=False
left=False
space=False
shift=False

mapping=('w','a','s','d','p','l', '')
# y+, x-, y-, x+, z+, z-, stop

def on_press(key):
    global up,down,left,right,space,shift
    
    if key==mapping[0]:
        print('y+ recorded')
        up=True
        down=False
    elif key==mapping[2]:
        print('y- recorded')
        down=True
        up=False
    elif key==mapping[3]:
        print('x+ recorded')
        right=True
        left=False
    elif key==mapping[1]:
        print('x- recorded')
        left=True
        right=False
    elif key==mapping[4]:
        print('z+ recorded')
        space=True
        shift=False
    elif key==mapping[5]:
        print('z- recorded')
        shift=True
        space=False
    elif key==mapping[6]:
        print('stop motion')
        up=False
        down=False
        left=False
        right=False
        space=False
        shift=False


def update_vec():
    vec[2]=float(space-shift)
    vec[0]=float(right-left)
    vec[1]=float(up-down)
    
def take_input():
    direct=input('direction:')
    if direct in mapping:
        on_press(direct)
    else:
        print('choose from',mapping)
    
while ACTIVE:
    take_input()
    update_vec()
    bb.move_agent(vec)
    time.sleep(.01)
    
    