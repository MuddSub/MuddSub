#!/usr/bin/env python3 
from tqdm.notebook import tqdm    # To get nice progress bars
from collections import deque
import gym
import math
import numpy as np
import random
import statistics
import tensorflow as tf
import collections
from gym.envs.registration import register


# You shouldn't need to change this class or alter the default values of the constructor

from tensorflow.python.framework.ops import disable_eager_execution

disable_eager_execution() # Haven't figured out why this is necessary but runs
                          # much slower without it

# this is our Deep Q learning network
class FunctionApproximator:
  """ Note that this function approximator will approximate q values for states.

  Rather than approximating q(state, action) directly, it approximates a function f(state) that returns
  the q values for *all* actions. It assumes that the actions are integers from 0..n-1.

  f(s)[1] contains the q value for action 1
  and f(s)[0] contains the q falue for action 0.

  To set the Q values for a state, call setQValuesForStates."""

  def __init__(self, num_actions=6, state_length=12, alpha=0.01, alpha_decay=0.01, batchsize=256):
    # initialization
    self.alpha = alpha
    self.alpha_decay = alpha_decay
    self.batchsize = batchsize
    self.num_actions = num_actions
    self.state_length = state_length

    # creating the neural network
    # state_length -> 24 -> 48 -> num_actions
    self.model = tf.keras.models.Sequential()
    self.model.add(tf.keras.layers.Dense(24, input_dim=self.state_length, activation='relu'))
    self.model.add(tf.keras.layers.Dense(48, activation='relu'))
    self.model.add(tf.keras.layers.Dense(self.num_actions, activation='linear'))

    self.model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(lr=self.alpha, decay=self.alpha_decay))

  def getQValuesForState(self, state):
    # given a state, we use the DQN to predict Q values of the all the actions
    """ state is a NumPy array with shape (1, state_length).

    Returns a (NumPy) array of Q-values for that state, indexed by action"""
    return self.model.predict(state)[0]  #[[0,1,2,3,4]]

  def getQValuesForStates(self, states):
    """Gets the Q values for a list of states. Each state is a NumPy array with shape (1, state_length).
    Returns it as a numpy array.
    The  array of Q values for the ith state can be accessed with
    getQValuesForStates([state0, state1, ..., statei, ..., staten])[i]

    It is much quicker to call this than to call getQValuesForState one-by-one."""
    if not isinstance(states, list):
      raise ValueError(f'states should be a Python list')
    for state in states:
      assert state.shape == (1, self.state_length)
    return self.model.predict(np.vstack(states), batch_size=len(states))

  def setQValuesForStates(self, states, qValues):
    """Sets the qValues for the given states.  states is a Python list of k states.
    qValues is a numpy 2-dimensional array.
    qValues is a parallel NumPy array of shape (k, numActions)
    So, qValues[i] is a numpy array of qValues for states[i].

    It is much quicker to call this on a number of states/qValues than to call
    multiple times with single state/qValue."""

    if not isinstance(states, list):
      raise ValueError(f'states should be a Python list')
    if not isinstance(qValues, np.ndarray):
      raise ValueError(f'qValues should be a NumPy array')
    if len(states) != qValues.shape[0]:
      raise ValueError(f'len(states) != qValues.shape[0]) ({len(states)} != {qValues.shape[0]})')
    if qValues.shape != (len(states), self.num_actions):
      raise ValueError(f'qValues.shape should be ({len(states)}, self.num_actions) but it is {qValues.shape} ')
    if len(states) < 5:
      print(f'Warning: setQValuesForStates works faster if you pass in large numbers of states and qValues at a time (you passed in only {len(states)}).')

    self.model.fit(np.vstack(states), qValues, batch_size=min(self.batchsize, len(states)), verbose=0)

class ExperienceBuffer:
  """Stores a buffer for doing replay of experiences.

  This acts like a model for doing planning."""

  def __init__(self, approximator, num_actions=2, state_length=4):
    self.approximator = approximator
    max_capacity = 10000
    # Use a deque which atuomatically pops front values when length gets too big
    self.dq = collections.deque([], maxlen = max_capacity)


  def remember(self, s, action, reward, sprime, done):
    """Stores this transition in the experience buffer."""
    # s' -> sprime -> next state
    self.dq.append(np.array([s, action, reward, sprime, done]))


  def plan(self):
    """ Runs through a random subsample of remembered transitions and does
    Q-learning on them."""
    subsample_len = 1000

    # do random sampling
    samples = random.sample(self.dq, min(subsample_len,len(self.dq)))


    #col 0: states, col1: actions, col2: rewards, col 3: sprimes(next state), col4: done

    samples_copy = np.array(samples)
    states = [np.array(state).reshape(-1,4) for state in samples_copy[:,0]]

    values = self.approximator.getQValuesForStates(states).reshape(-1,2)

    new_states = [np.array(state).reshape(-1,4) for state in samples_copy[:,3]]
    new_values = np.array(self.approximator.getQValuesForStates(new_states)).reshape(-1,2)
    done = samples_copy[:,4].reshape(-1,1)

    # r + max_a'(q(s',a')) -> get the new value
    updated_q = samples_copy[:,2] + np.max(new_values, axis=1)
    actions = samples_copy[:,1]

    # Get updated q[s,a] pairs
    for idx, value in enumerate(values):
      # Update values of correct action
      values[idx][actions[idx]] = updated_q[idx]
      if done[idx]:
        # If done, q=r=0
        values[idx][actions[idx]] = 0

    # Fit the model -> train the nn with the new value
    self.approximator.setQValuesForStates(states,values)

class Solver:
  """Solves the Cartpole problem."""
  def __init__(self, env, approximator, experience_buffer, epsilon_min=0.01):
    self.env = env
    self.approximator = approximator
    self.experience_buffer = experience_buffer
    self.epsilon_min = epsilon_min

  def getAction(self, state, epsilon):
    """Returns the action to take from the given state.  Uses passed-in epsilon
    for epsilon-greedy action."""
    # TODO: You'll need to do epsilon-greedy, and will need to use the approximator
    # to determine the greedy action
    if random.random() > epsilon:
      values = self.approximator.getQValuesForState(state.reshape(1,4))
      action = np.argmax(values)
      return action
    return self.env.action_space.sample() # Returns a random action

  def getEpsilon(self, episodeNum):
    """Returns the epsilon to use for the given episode number."""
    # Exponential decay epsilon
    return max(0.99**episodeNum, self.epsilon_min)

  def train(self, num_episodes=5):
    """ Uses the environment to train on CartPole for ```num_episodes```."""
    count = 0
    for episode in range(num_episodes):
      print(f'Starting episode {episode}')
      observation = env.reset()
      for i in range(10):
        action = self.getAction(observation, self.getEpsilon(episode))
        new_observation, reward, done, info = env.step(action)
        # Save experience
        self.experience_buffer.remember(observation, action, reward, new_observation, done)
        # Do q learning
        self.experience_buffer.plan()
        observation = new_observation

    return self.approximator

  def play(self, num_episodes=10):
    """ Uses the environment to run CartPole for ```num_episodes```.  Doesn't do any learning, just
    takes greedy actions as specified by the approximator."""
    lengths = []
    for episode in range(num_episodes):
      observation = env.reset()
      for i in range(10):
        action = self.getAction(observation, 0)
        #print(f'transition on state {observation}, action {action}')
        observation, reward, done, info = env.step(action)
        observation= np.array(observation).reshape(-1,4)
        #print(f' resulted in new state {observation}, reward {reward}')
        if done:
          lengths.append(i)
          print(f'Finished in {i} steps')
          break
    '''
    plt.rcParams.update({'font.size': 15})
    plt.scatter(np.arange(len(lengths)),lengths)
    plt.title("Episode Lengths with Converged Policy")
    plt.xlabel("Episode Number")
    plt.ylabel("Timesteps")
    print(f"Average steps {np.average(lengths)}")
    '''

env = gym.make('MuddSub-v0')
approximator = FunctionApproximator(num_actions=6, state_length=12)
experience_buffer = ExperienceBuffer(approximator)
solver = Solver(env, approximator, experience_buffer)
trained_approximator = solver.train()
'''
env = gym.make('CartPole-v1')
experience_buffer = ExperienceBuffer(trained_approximator)
solver = Solver(env, trained_approximator, experience_buffer)
solver.play()
'''
