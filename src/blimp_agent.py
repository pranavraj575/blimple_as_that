import subprocess, psutil
import time, os, sys
import rclpy
import numpy as np

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64

from CONFIG import *

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))

msgfile = os.path.join(DIR, 'lua', 'rosMsg.lua')
TOPIC_NAMES = dict()
with  open(msgfile) as f:
    r = [t.split('=') for t in f.read().strip().split('\n') if '=' in t]
    for key, item in r:
        TOPIC_NAMES[key.strip()] = item.strip().strip("'")

frictionless_wall_path = os.path.join(DIR, 'scenes', 'FrictionlessWallClimb.ttt')
wall_climb_path = os.path.join(DIR, 'scenes', 'WallClimb.ttt')
wall_climb_undetect_path = os.path.join(DIR, 'scenes', 'WallClimbUndetectable.ttt')
caged_wall_climb_path = os.path.join(DIR, 'scenes', 'wall_climb_caged.ttt')
cage_arena_path = os.path.join(DIR, 'scenes', 'cage_arena_better.ttt')
empty_path = os.path.join(DIR, 'scenes', 'empty.ttt')

narrow_blimp_path = os.path.join(DIR, 'ros_ctrl_models', 'blimp_narrow.ttm')

def init_sim(simId):
    from zmqRemoteApi import RemoteAPIClient
    # NECESSARY to make this work with multiple simulators
    client = RemoteAPIClient(port=simId)
    sim = client.getObject('sim')
    return sim

def moveObject(sim, handle, pos, orient=None):
    """

    @param sim: simulation
    @param handle: object handle
    @param pos: position 
    @param orient: orientation
        roll, pitch, yaw
    """
    Pos = []
    
    for k in range(3):
        Pos.append(float(pos[k]))
        
    sim.setObjectPosition(handle, -1, Pos)
    if orient is not None:
        Orient = []
        for k in range(3):
            Orient.append(float(orient[k]))
        sim.setObjectOrientation(handle, -1, Orient)

def spawnModel(sim,modelPath, pos, orient=None):
    """
    spawns a model in a certian area
    @param sim: simulation to spawn it in
    @param modelPath: path to .ttm model to spawn
    @param pos: position
    @param orient: orientation
        roll, pitch, yaw
    @return: handle of model spawned
    """
    agentHandle = sim.loadModel(os.path.abspath(os.path.expanduser(modelPath)))
    
    moveObject(sim=sim,handle=agentHandle, pos=pos, orient=orient)
    
    return agentHandle

class BlimpAgent:

    def __init__(self,
                 blimpPath=narrow_blimp_path,
                 sim=None,
                 msg_queue=10,
                 agent_id=0,
                 simId=23000,
                 ):
        """
        blimp agent (controlled by ROS velocity controller)

        @param blimpPath: path to blimp for spawning
        @param sim: zqm simulator api, if None, than makes its own
        @param msg_queue: queue length of ROS messages
        @param agent_id: identifier number of the blimp, must be unique among all blimps (usually blimp number)
        @param simId: simulator id where the blimp should spawn, used to pass messages to correct topics
        """
        self.msg_queue = msg_queue
        self.modelPath = blimpPath
        self.agentData = dict()
        self.agent_id=agent_id
        self.simId=simId
        
        if sim is not None:
            self.sim = sim
        else:
            self.sim=init_sim(simId)
        
    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawnAgent(self,position):
        """
        @param position: where to spawn it
        """
        TOPIC_PRE_BLIMP = TOPIC_NAMES['TOPIC_PRE_BLIMP']
        TOPIC_CMD = TOPIC_NAMES['TOPIC_CMD']
        TOPIC_GLOBAL = TOPIC_NAMES['TOPIC_GLOBAL']
        TOPIC_ULTRA = TOPIC_NAMES['TOPIC_ULTRA']

        if not rclpy.ok():
            rclpy.init()
        
        this_agent = dict()
        this_agent['agentHandle'] = spawnModel(sim=self.sim,modelPath=self.modelPath, pos=position)
        
        i=self.agent_id
        this_agent['agent_id'] = i

        unique = str(time.time()).replace('.', '_')
        NODE = rclpy.create_node('lta_' + str(self.simId) + '_' + str(i) + '_NODE_' + unique)
        this_agent['NODE'] = NODE

        cmd_topic = TOPIC_PRE_BLIMP + str(self.simId) + '_' + str(i) + TOPIC_CMD
        this_agent['cmd_topic'] = cmd_topic

        state_topic = TOPIC_PRE_BLIMP + str(self.simId) + '_' + str(i) + TOPIC_GLOBAL
        this_agent['state_topic'] = state_topic

        ultra_topic = TOPIC_PRE_BLIMP + str(self.simId) + '_' + str(i) + TOPIC_ULTRA
        this_agent['ultra_topic'] = ultra_topic

        vec_publisher = NODE.create_publisher(Twist, cmd_topic, self.msg_queue)
        this_agent['vec_publisher'] = vec_publisher

        callback = self.create_callback_twist(this_agent, 'state')
        state_subscriber = NODE.create_subscription(TwistStamped,
                                                    state_topic,
                                                    callback,
                                                    self.msg_queue)
        this_agent['state_subscriber'] = state_subscriber

        ultracallback = self.create_callback_float(this_agent, 'state')
        ultra_subscriber = NODE.create_subscription(Float64,
                                                    ultra_topic,
                                                    ultracallback,
                                                    self.msg_queue)
        this_agent['ultra_subscriber'] = ultra_subscriber

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(NODE)
        this_agent['executor'] = executor
        
        self.agentData = this_agent

    def despawnAgent(self):
        """
        to remove the ros nodes

        @note: FOR SOME REASON, it works better to not delete the nodes, and just leave them as warnings
        """
        return
    
        for sub_key in ('state_subscriber', 'ultra_subscriber'):
            self.agentData['NODE'].destroy_subscription(sub_key)
        for pub_key in ('vec_publisher',):
            self.agentData['NODE'].destroy_publisher(pub_key)
        self.agentData['NODE'].destroy_node()

    ####################################################################################################################
    # ROS functions
    ####################################################################################################################
    def spin(self):
        """
        spins the sensor node

        """
        self.agentData['executor'].spin_once(timeout_sec=.01)

    def create_callback_twist(self, dictionary, key, state_keys=('x', 'y', 'z', 'w', 'DEBUG')):
        """
        creates a callback that updates the "key" element of "dictionary" with the twist state

        @param dictionary: dictionary to update
        @param key: key in dictionary to update
        @param state_keys: keys to put x,y,z,w,DEBUG values
        @return: returns callback function to be used in ROS subscriber
        """
        if key not in dictionary:
            dictionary[key] = dict()
        # default values of 0
        dictionary[key].update({k: 0. for k in state_keys})

        def callback(msg):
            dictionary[key][state_keys[0]] = msg.twist.linear.x
            dictionary[key][state_keys[1]] = msg.twist.linear.y
            dictionary[key][state_keys[2]] = msg.twist.linear.z
            dictionary[key][state_keys[3]] = msg.twist.angular.z
            dictionary[key][state_keys[4]] = 1.

        return callback

    def create_callback_float(self, dictionary, key, state_key='ultra', debug_key="ULTRA_DEBUG"):
        """
        creates a callback that updates the "key" element of "dictionary" with the twist state

        @param dictionary: dictionary to update
        @param key: key in dictionary to update
        @param state_key: key to put float values
        @param debug_key: key to put debug stuff (currently whether callback is run)
        @return: returns callback function to be used in ROS subscriber
        """
        if key not in dictionary:
            dictionary[key] = dict()
        # default value of 0
        dictionary[key].update({state_key: 0., debug_key: 0.})

        def callback(msg):
            dictionary[key][state_key] = msg.data
            dictionary[key][debug_key] = 1.

        return callback

    ####################################################################################################################
    # Agent functions
    ####################################################################################################################

    def move_agent(self, vec):
        """
        publishes a vector to agent
            (currently publishes a velocity goal, and LUA controls in blimpNew.lua takes care of rest)

        @param vec: vector to publish

        @note: currently using just the linear part of twist message,
            can use orientation for other stuff if we update blimpNew.lua
        """
        msgTwist = Twist()
        msgTwist.linear.x = float(vec[0])
        msgTwist.linear.y = float(vec[1])
        msgTwist.linear.z = float(vec[2])

        self.agentData['vec_publisher'].publish(msgTwist)

    def is_connected(self):
        """
        returns if the agent state has been read by ROS

        @return: boolean on if state has been seen
        """
        s = self.get_state(spin=False)
        return bool(s["DEBUG"])

    def get_state(self, spin=True):
        """
        returns state of agent

        @param spin: whether to update agent before getting state
        @rtype: dictionary
        @return: state of agent
        """
        if spin:
            self.spin()
        return self.agentData['state']

    def get_position(self, use_ultra=False, spin=True):
        """
        returns position of agent

        @param use_ultra: whether to use ultrasound sensor as opposed to state['z']
        @param spin: whether to update agent before getting state
        @rtype: R^3 numpy array
        @return: position of agent
        """
        s = self.get_state(spin=spin)
        return np.array((s['x'], s['y'], s['ultra' if use_ultra else 'z']))


if __name__ == "__main__":
    import time
    bb=BlimpAgent(blimpPath=narrow_blimp_path)
    bb.spawnAgent((0,0,1))
    bb.sim.startSimulation()
    time.sleep(1)
    bb.move_agent((0,0,.5))
    time.sleep(3)
    bb.sim.stopSimulation()
    
    
