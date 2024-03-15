import pygame
import sys, os
sys.path.append("..")
from src.blimp_agent import BlimpAgent
import rclpy
from rclpy.node import Node
from zmqRemoteApi import RemoteAPIClient
import os
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import numpy as np


PI = 3.14159
SIMID = 23000

path = os.getcwd()
DIR = os.path.dirname(path)

#Set up new simulation
msgfile = os.path.join("../", 'lua', 'rosMsg.lua')
TOPIC_NAMES = dict()
with  open(msgfile) as f:
    r = [t.split('=') for t in f.read().strip().split('\n') if '=' in t]
    for key, item in r:
        TOPIC_NAMES[key.strip()] = item.strip().strip("'")

TOPIC_PRE = TOPIC_NAMES['TOPIC_PRE_BLIMP']
TOPIC_CMD = TOPIC_NAMES['TOPIC_CMD']
TOPIC_GLOBAL = TOPIC_NAMES['TOPIC_GLOBAL']

state = {
    'x': 0,
    'y': 0,
    'z': 0,
    'w': 0
}

RECCY = False
POS = []


def callbackUpdateState(msg):
    global state
    global POS
    print('here')
    state['x'] = msg.twist.linear.x
    state['y'] = msg.twist.linear.y
    state['z'] = msg.twist.linear.z
    state['w'] = msg.twist.angular.z
    if RECCY:
        POS.append(np.array([state['x'], state['y'], state['z'], state['w']]))
    # print(state)



def main(args=None):
    global state

    pygame.init()
    window = pygame.display.set_mode((300, 300))
    clock = pygame.time.Clock()

    rect = pygame.Rect(0, 0, 20, 20)
    rect.center = window.get_rect().center
    vel = 5


    client = RemoteAPIClient()
    sim = client.getObject('sim')

    RESET = True

    MODELDIR = os.path.join(DIR, 'ros_ctrl_models', 'blimp_narrow.ttm')
    SCENEDIR = os.path.join(DIR, 'scenes', 'empty.ttt')

    narrowModelPath = os.path.abspath(os.path.expanduser(MODELDIR))
    modelToLoad = narrowModelPath

    if RESET:
        sceneNamePath = os.path.abspath(os.path.expanduser(SCENEDIR))
        sim.stopSimulation()
        time.sleep(1)
        print(sceneNamePath)
        sim.loadScene(sceneNamePath)
        time.sleep(1)

        agentHandle = sim.loadModel(modelToLoad)
    agent = '0'

    topicCmdVel = TOPIC_PRE + str(SIMID) + '_' + agent + TOPIC_CMD
    topicGlobal = TOPIC_PRE + str(SIMID) + '_' + agent + TOPIC_GLOBAL
    print(topicGlobal)
    rclpy.init(args=args)

    NODE = rclpy.create_node('test')
    publisherAlign = NODE.create_publisher(Twist, topicCmdVel, 10)
    subscriberPos = NODE.create_subscription(TwistStamped, topicGlobal, callbackUpdateState, 10)

    DT = 50/1000
    sim.startSimulation()
    test = np.array([.0, .0, .05, np.pi])
    f = 1.
    i = 0

    run = True
    while run:
        #clock.tick(600)


        #Default punlishing
        msgTwist = Twist()
        msgTwist.linear.x = 0.
        msgTwist.linear.y = 0.
        msgTwist.linear.z = 0.0
        msgTwist.angular.z = 0.
        msgTwist.angular.x = 1.  # NOTE: this tells the blimp that we care about heading
        #


        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
                print(pygame.key.name(event.key))

                #publisherAlign.publish(msgTwist)

                if pygame.key.name(event.key) == "up":
                    msgTwist.linear.z = 1.

                if pygame.key.name(event.key) == "down":
                    msgTwist.linear.z = -1.

                if pygame.key.name(event.key) == "left":
                    msgTwist.angular.z = -10.

                if pygame.key.name(event.key) == "right":
                    msgTwist.angular.z = 10.

                if pygame.key.name(event.key) == "w":
                    msgTwist.linear.x = 1.

                if pygame.key.name(event.key) == "s":
                    msgTwist.linear.x = -1.

                if pygame.key.name(event.key) == "a":
                    msgTwist.linear.y = -1.

                if pygame.key.name(event.key) == "d":
                    msgTwist.linear.y = 1.

            publisherAlign.publish(msgTwist)

        #publisherAlign.publish(msgTwist)


        keys = pygame.key.get_pressed()

        #print(pygame.K_UP)

        #print(keys)

        #rect.x += (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * vel
        #rect.y += (keys[pygame.K_DOWN] - keys[pygame.K_UP]) * vel

        #rect.centerx = rect.centerx % window.get_width()
        #rect.centery = rect.centery % window.get_height()

        window.fill(0)
        #pygame.draw.rect(window, (255, 0, 0), rect)
        pygame.display.flip()

    pygame.quit()
    exit()

    sim.pauseSimulation()
    sim.stopSimulation()



if __name__ == '__main__':
    main()
