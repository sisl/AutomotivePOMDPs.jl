import numpy as np
import pdb
import time
import logging
# import universe
from scipy.ndimage.interpolation import shift
import zmq
import gym
from gym import spaces
from collections import deque


class ZMQConnection:

    def __init__(self, ip, port):
        self._ip = ip
        self._port = port

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.connect("tcp://{}:{}".format(ip, port))

    @property
    def socket(self):
        return self._socket

    def sendreq(self, msg):
        self.socket.send_json(msg)
        respmsg = self.socket.recv_json()
        return respmsg

class LazyFrames(object):
    def __init__(self, frames):
        """This object ensures that common frames between the observations are only stored once.
        It exists purely to optimize memory usage which can be huge for DQN's 1M frames replay
        buffers.

        This object should only be converted to numpy array before being passed to the model.

        You'd not belive how complex the previous solution was."""
        self._frames = frames

    def __array__(self, dtype=None):
        out = np.concatenate(self._frames, axis=2)
        if dtype is not None:
            out = out.astype(dtype)
        return out

class FrameStack(gym.Wrapper):
    def __init__(self, env, k):
        """Stack k last frames.

        Returns lazy array, which is much more memory efficient.

        See Also
        --------
        baselines.common.atari_wrappers.LazyFrames
        """
        gym.Wrapper.__init__(self, env)
        self.env = env
        self.to_log = env.to_log
        self.k = k
        self.frames = deque([], maxlen=k)
        shp = env.observation_space.shape
        self.action_space = env.action_space
        self.observation_space = spaces.Box(low=-10., high=10., shape=(shp[0], shp[1], shp[2] * k))

    def reset(self):
        ob = self.env.reset()
        for _ in range(self.k):
            self.frames.append(ob)
        return self._get_ob()

    def step(self, action):
        ob, reward, done, info = self.env.step(action)
        self.frames.append(ob)
        return self._get_ob(), reward, done, info

    def _get_ob(self):
        assert len(self.frames) == self.k
        return LazyFrames(list(self.frames))

class ScaledFloatFrame(gym.ObservationWrapper):
    def __init__(self, env):
        gym.ObservationWrapper.__init__(self, env)
        self.to_log = env.to_log

    def _observation(self, obs):
        # careful! This undoes the memory optimization, use
        # with smaller replay buffers only.
        return np.array(obs).astype(np.float32)

class ActionSpace(object):
    def __init__(self, n):
        self.n = n

    def sample(self):
        return np.random.randint(0, self.n)

class ObservationSpace(object):
    def __init__(self, shape):
        self.shape = shape

class OccludedCrosswalk(gym.Env):

    def __init__(self, ip='127.0.0.1', port=9393):
        self._conn = ZMQConnection(ip, port)
        self.running_average = np.ones(1000)
        # uncomment below for image representation
        # self.observation_space = ObservationSpace((65,21,4))
        # uncomment below for vector representation
        self.observation_space = ObservationSpace((4,1,1))
        self.action_space = ActionSpace(4)
        self.time_limit = 100
        self.metadata = {}

        # for diagnostic
        self.to_log = {}
        self.total_episodes = 0
        self.total_crashes = 0
        self._episode_time = time.time()
        self._last_time = time.time()
        self._local_t = 0
        self._episode_reward = 0
        self._episode_length = 0
        self._all_rewards = []
        self._num_vnc_updates = 0
        self._last_episode_id = -1

    def step(self, action):
        data = self._conn.sendreq({"cmd": "step", "args": int(action + 1)})
        assert 'obs' in data
        assert 'rew' in data
        assert 'done' in data
        assert 'info' in data
        o, r, done, info = np.reshape(data['obs'], self.observation_space.shape) , data['rew'], data['done'], data['info']
        if self._episode_length > self.time_limit:
            done = True
        self._after_step(o, r, done, info)
        return o, r, done, info

    def _after_step(self, observation, reward, done, info):
        if self._episode_length == 0:
            self._episode_time = time.time()

        self._local_t += 1


        if reward is not None:
            self._episode_reward += reward
            if observation is not None:
                self._episode_length += 1
            self._all_rewards.append(reward)

        if done:
            self.total_episodes += 1
            # logger.info('Episode terminating: episode_reward=%s episode_length=%s', self._episode_reward, self._episode_length)
            total_time = time.time() - self._episode_time
            self.to_log["global/episode_reward"] = self._episode_reward
            self.to_log["global/episode_length"] = self._episode_length
            # to_log["global/episode_time"] = total_time
            # to_log["global/reward_per_time"] = self._episode_reward / total_time
            crash = 1. if reward <= -1.0 else 0.
            self.running_average = shift(self.running_average, -1, cval=crash)
            self.total_crashes += crash
            self.to_log["global/episodes_crash"] = crash
            self.to_log["global/collision_rate"] = float(self.total_crashes)/float(self.total_episodes)
            self.to_log["global/running_average"] = np.sum(self.running_average)/10.0
            self._episode_reward = 0
            self._episode_length = 0
            self._all_rewards = []

    def reset(self):
        data = self._conn.sendreq({"cmd": "reset"})
        assert 'obs' in data
        return np.reshape(data['obs'], self.observation_space.shape)

    def _after_reset(self, observation):
        # logger.info('Resetting environment')
        self._episode_reward = 0
        self._episode_length = 0
        self._all_rewards = []
        return observation

    # @property
    # def action_space(self):
    #     data = self._conn.sendreq({"cmd": "n_actions"})
    #     assert 'n_actions' in data
    #     return spaces.Discrete(data['n_actions'])
    #
    # @property
    # def observation_space(self):
    #     data = self._conn.sendreq({"cmd": "obs_dimensions"})
    #     assert 'obs_dim' in data
    #     pdb.set_trace()
    #     return (data['obs_dim'][0],1,1)




if __name__ == '__main__':
    env = OccludedCrosswalk()
    env = FrameStack(env, 1)
    obs = env.reset()
    while True:
        action = env.action_space.sample()
        ob, reward, done, _ = env.step(action)

        # print("s ->{}".format(ob.__array__(np.float32)))
        # print("a ->{}".format(action))
        # print("sp->{}".format(ob.__array__(np.float32)))
        # print("r ->{}".format(reward))

        obs = ob
        if done:
            print("r -> {}".format(reward))
            break

    env.close()
