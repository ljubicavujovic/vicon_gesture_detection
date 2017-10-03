import numpy as np
from scipy.stats import multivariate_normal
from speech import Speech

class BayesFilter:
    """
    Bayes Filter is used for updating belief state with new observations from
    speech and gesture.
    """
    def __init__(self, transition_model, observation_model=None, states=None):
        self.transition_model = transition_model
        self.observation_model = observation_model
        self.states_number = transition_model.shape[0]
        self.belief = np.zeros((self.states_number, 1))
        if states is None:
            self.belief.fill(1./self.states_number)
        else:
            self.belief = states
        self.speech = Speech()

    def prediction(self, observation):
        self.belief = observation * np.dot(self.transition_model.T, self.belief)
        self.belief = self.normalize(self.belief)
        return self.belief

    def normalize(self, states):
        return states/sum(states)

    def gesture_observation(self, coordinate_difference, sigma=np.array([1, 1, 1])):
        mu = np.array([0.0, 0.0, 0.0])
        covariance = np.diag(sigma ** 2)

        observation = np.zeros((self.states_number, 1))
        observation = multivariate_normal.pdf(coordinate_difference, mu, covariance)
        observation = observation.reshape((self.states_number, 1)).T

        return observation.T

    def speech_observation(self, coordinates, sentance):
        self.speech.coordinates = coordinates
        return self.speech.observation_distribution(sentance)

    def reset(self):
        self.belief.fill(1./self.states_number)
