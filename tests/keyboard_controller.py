from src.blimp_agent import *

from pynput.keyboard import Key, Listener
import numpy as np
import time

bb=BlimpAgent(blimpPath=narrow_blimp_path)
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

def on_press(key):
    global up,down,left,right,space,shift
    
    if key==Key.up:
        print('y+ recorded')
        up=True
    elif key==Key.down:
        print('y- recorded')
        down=True
    elif key==Key.left:
        print('x+ recorded')
        left=True
    elif key==Key.right:
        print('x- recorded')
        right=True
    elif key==Key.space:
        print('z+ recorded')
        space=True
    elif key==Key.shift:
        print('z- recorded')
        shift=True
    
def on_release(key):
    global up,down,left,right,space,shift,ACTIVE
    
    if key==Key.up:
        up=False
    elif key==Key.down:
        down=False
    elif key==Key.left:
        left=False
    elif key==Key.right:
        right=False
    elif key==Key.space:
        space=False
    elif key==Key.shift:
        shift=False
    elif key==Key.esc:
        ACTIVE=False

def update_vec():
    vec[2]=float(space-shift)
    vec[0]=float(right-left)
    vec[1]=float(up-down)

while ACTIVE:
    update_vec()
    bb.move_agent(vec)
    time.sleep(.01)
    
    