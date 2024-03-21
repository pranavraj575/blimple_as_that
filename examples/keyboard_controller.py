import sys
sys.path.append("..")

from src.blimp_agent import BlimpAgent

from pynput.keyboard import *
import numpy as np
import time

bb=BlimpAgent()
bb.spawnAgent((0,0,1))
bb.sim.startSimulation()
ACTIVE=True

vec=np.zeros(4)


up=False
down=False
right=False
left=False
space=False
shift=False

print(KeyCode.from_char('w'))

mapping=(KeyCode.from_char('w'),KeyCode.from_char('a'),KeyCode.from_char('s'),KeyCode.from_char('d'),Key.space,Key.shift, Key.left, Key.right)
# y+, x-, y-, x+, z+, z-


def on_press(key):
    global up,down,left,right,space,shift

    print(key, mapping[0])
    
    if key==KeyCode.from_char('w'):
        print('y+ recorded')
        up=True
    elif key==mapping[2]:
        print('y- recorded')
        down=True
    elif key==mapping[3]:
        print('x+ recorded')
        right=True
    elif key==mapping[1]:
        print('x- recorded')
        left=True
    elif key==mapping[4]:
        print('z+ recorded')
        space=True
    elif key==mapping[5]:
        print('z- recorded')
        shift=True
    
def on_release(key):
    global up,down,left,right,space,shift,ACTIVE
    
    if key==mapping[0]:
        up=False
    elif key==mapping[2]:
        down=False
    elif key==mapping[3]:
        right=False
    elif key==mapping[1]:
        left=False
    elif key==mapping[4]:
        space=False
    elif key==mapping[5]:
        shift=False
    elif key==Key.esc:
        ACTIVE=False

def update_vec():
    vec[2]=float(space-shift)
    vec[0]=float(right-left)
    vec[1]=float(up-down)
    vec[3] = float(up - down)
    
listen=Listener(on_press=on_press,on_release=on_release)
listen.start()
while ACTIVE:
    update_vec()
    bb.move_agent(vec)
    time.sleep(.01)
    
    