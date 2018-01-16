import numpy as np
import dill
import subprocess
from dqn.rename_tensorflow_variables import rename
from baselines import logger
from baselines.deepq import build_act
from baselines.deepq.simple import ActWrapper
import dqn.baseline_graph
import baselines.common.tf_util as U
import tensorflow as tf
import sys
import argparse
import pdb

def callback(lcl, glb):
    # extra logging and evaluation
    if lcl['t'] == 0:
        return False
    log_freq = 10
    # pdb.set_trace()
    eval_freq = 1000
    num_eval_ep = 100
    env = lcl['env']
    # if lcl['t'] % log_freq == 0:
    #     # for key, value in env.env.env.to_log.items():
        # for key, value in env.env.env.to_log.items():
        #     logger.record_tabular(key, value)
    if lcl['done'] and lcl['num_episodes'] % eval_freq == 0:
        logger.info('Evaluation...')
        rewards, steps, crashes, action_stats_glob = [], [], 0, np.zeros(env.action_space.n)
        crashes, successes, time_out = 0, 0, 0
        for ep in range(0, num_eval_ep):
            action_stats = np.zeros(env.action_space.n)
            obs, step, rtot, done = env.reset(), 0, 0., False
            while not done: #parameterized
                action = lcl['act'](obs[None])[0]
                obs, rew, done, _ = env.step(action)
                rtot += rew
                step += 1
                action_stats[action] += 1
            if step==102: #step == max_steps:
                # print("time_out {}".format(step))
                time_out += 1
            elif rtot <= -1.0 : # action cost, TODO parameterized
                crashes += 1
            else:
                successes += 1
            action_stats /= np.sum(action_stats)/100
            action_stats_glob += action_stats
            rewards.append(rtot)
            steps.append(step)
        action_stats_glob /= num_eval_ep
        avg_reward = np.mean(rewards)
        sigma_reward = np.sqrt(np.var(rewards) / len(rewards))
        avg_step = np.mean(steps)
        sigma_step = np.sqrt(np.var(steps) / len(steps))

        msg = "Average reward: {:04.2f} +/- {:04.2f} ; Average step: {:04.2f} +/- {:04.2f} ; Action stats (%) {} "\
              .format(avg_reward, sigma_reward, avg_step, sigma_step, action_stats_glob)
        logger.record_tabular("Eval/reward", avg_reward)
        logger.record_tabular("Eval/crashes", crashes)
        logger.record_tabular("Eval/time_out", time_out)
        logger.record_tabular("Eval/successes", successes)
        logger.info(msg)

    is_solved = sum(lcl['episode_rewards'][-1001:-1]) / 1000 == 1
    return is_solved

def evaluate(env, act, num_eval_ep = 1000, max_steps=102, verbose=False):
    rewards, steps, action_stats_glob = [], [], np.zeros(env.action_space.n)
    crashes, successes, time_out = 0, 0, 0
    for ep in range(0, num_eval_ep):
        action_stats = np.zeros(env.action_space.n)
        obs, step, rtot, done = env.reset(), 0, 0., False
        while not done:
            action = act(obs[None])[0]
            obs, rew, done, _ = env.step(action)
            rtot += rew
            step += 1
            action_stats[action] += 1
        # pdb.set_trace()
        if step==max_steps: #step == max_steps:
            # print("time_out {}".format(step))
            time_out += 1
        elif rtot <= -1.0: # action cost, TODO parameterized
            crashes += 1
        else:
            successes += 1
        action_stats /= np.sum(action_stats)/100
        action_stats_glob += action_stats
        if verbose:
            print("Episode {}; reward {}; action stats {}".format(ep, rtot, action_stats))
        rewards.append(rtot)
        steps.append(step)
    action_stats_glob /= num_eval_ep
    avg_reward = np.mean(rewards)
    sigma_reward = np.sqrt(np.var(rewards) / len(rewards))
    avg_step = np.mean(steps)
    sigma_step = np.sqrt(np.var(steps) / len(steps))
    avg_crashes = crashes/num_eval_ep*100
    avg_successes = successes/num_eval_ep*100
    avg_timeout = time_out/num_eval_ep*100

    msg = "Average reward: {:04.2f} +/- {:04.2f} ; Average crashes: {:04.2f}; Average step: {:04.2f} +/- {:04.2f} ; Action stats (%) {} "\
          .format(avg_reward, sigma_reward, avg_crashes, avg_step, sigma_step, action_stats_glob)

    # logger.record_tabular("Eval reward", avg_reward)
    logger.info(msg)
    return avg_reward, avg_step, avg_crashes, avg_successes, avg_timeout

def _evaluate(env, act, num_eval_ep = 500, max_steps=100, verbose=False):
    return evaluate(env, act, num_eval_ep, max_steps, verbose)[3]

def save_all(act, model, path):
    # save model
    model_path = path + 'model.pkl'
    with open(model_path, 'wb') as f:
        dill.dump(model, f)
    # save tf params
    tfmodel_path = path + 'model.ckpt'
    U.save_state(tfmodel_path)
    # rename
    rename(path, None, None, "saved/", None, True, False)



def restore_act_and_value(env, path, num_cpu=4, scope="saved/deepq", reuse=None):
    # pdb.set_trace()
    qfunc_path = path + 'model.pkl'
    with open(qfunc_path, "rb") as f:
        q_func = dill.load(f)
    def make_obs_ph(name):
        return U.BatchInput(env.observation_space.shape, name=name)
    act = build_act(make_obs_ph, q_func, env.action_space.n, scope, reuse)
    value = build_value_function(make_obs_ph, q_func, env.action_space.n, scope, True)
    sess = U.make_session(num_cpu=num_cpu)
    sess.__enter__()
    # for debugging
    # for var in tf.global_variables():
    #     print(var.name)
    U.load_state(tf.train.latest_checkpoint(path))
    act_params = {'make_obs_ph': make_obs_ph, 'q_func': q_func, 'num_actions': env.action_space.n}
    return ActWrapper(act, act_params), value


def build_value_function(make_obs_ph, q_func, num_actions, scope="deepq", reuse=None):
        """Creates the value function:
        My own version of baselines.build_act which returns the q_values instead of just the action

        Parameters
        ----------
        make_obs_ph: str -> tf.placeholder or TfInput
            a function that take a name and creates a placeholder of input with that name
        q_func: (tf.Variable, int, str, bool) -> tf.Variable
            the model that takes the following inputs:
                observation_in: object
                    the output of observation placeholder
                num_actions: int
                    number of actions
                scope: str
                reuse: bool
                    should be passed to outer variable scope
            and returns a tensor of shape (batch_size, num_actions) with values of every action.
        num_actions: int
            number of actions.
        scope: str or VariableScope
            optional scope for variable_scope.
        reuse: bool or None
            whether or not the variables should be reused. To be able to reuse the scope must be given.

        Returns
        -------
        value: (tf.Variable, bool, float) -> tf.Variable
            function to select and action given observation.
    `       See the top of the file for details.
        """
        with tf.variable_scope(scope, reuse=reuse):
            observations_ph = U.ensure_tf_input(make_obs_ph("observation"))

            q_values = q_func(observations_ph.get(), num_actions, scope="q_func")

            value = U.function(inputs=[observations_ph],
                             outputs=q_values)
            return value

def parse_hyperparam():
    """
    returns an object with the hyperparameters as attribute
    """
    parser = argparse.ArgumentParser(description="process hyperparameters and logdir")
    parser.add_argument('--logdir', dest='path', type=str,
                        default='test/',
                        help="specify directory to log the weights")
    parser.add_argument('--update-freq', dest='update_freq', type=int,
                        default=5000,
                        help="frequency at which to update the target network")
    parser.add_argument('--exp-fraction', dest='exploration_fraction', type=float,
                        default=0.8,
                        help="fraction of the training set used for exploration")
    parser.add_argument('--final-eps', dest="final_eps",type=float,
                        default=0.01,
                        help="final value of epsilon for epsilon greedy exploration")
    parser.add_argument('--num-nodes', dest="num_nodes",  type=int,
                        default=128,
                        help="number of nodes to use in the hidden layers")
    parser.add_argument('--port', dest="port",  type=int,
                        default=9393,
                        help="port for zmq communication")
    parser.add_argument('--correction', dest="correction", type=str,
                        default=False,
                        help="load the single policy graph from the given folder \
                              and use it as a baseline")
    parser.add_argument('--fusion', dest="fusion", type=str, default="min",
                        help="choose the function used to fuse the utilities")
    parser.add_argument('--maxsteps', dest="maxsteps", type=int, default=1000000,
                        help="number of examples to train on")
    args = parser.parse_args()
    return args
