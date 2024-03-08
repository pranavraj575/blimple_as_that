from src.blimp_agent import BlimpAgent

from pynput.keyboard import Key, Listener
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

def on_press(key):
    global up,down,left,right,space,shift
    
    if key==Key.up:
        up=True
    elif key==Key.down:
        down=True
    elif key==Key.left:
        left=True
    elif key==Key.right:
        right=True
    elif key==Key.space:
        space=True
    elif key==Key.shift:
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
    
    