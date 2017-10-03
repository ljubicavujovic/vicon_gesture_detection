#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from scipy.stats import multivariate_normal
from bayes_filter import BayesFilter
from speech import Speech
from sys import stdin

# fast forgetting -> c lower
# still system -> c higher
c = 0.95
transition_model = np.zeros((5, 5))
transition_model.fill(1.*(1-c)/(transition_model.shape[0]-1))
np.fill_diagonal(transition_model, c)
BF = BayesFilter(transition_model)

class BayesListener:
    def __init__(self):
        self.subscriber_gesture = rospy.Subscriber("coordinates", Float64MultiArray, self.callback_gesture)
        self.subscriber_speech = rospy.Subscriber("speech", String, self.callback_speech)
        self.publisher_alexa = rospy.Publisher("alexa_in", String, queue_size=10)
        self.coordinates = np.zeros((5,3))
        self.coor_diff = np.zeros((5,3))

    def callback_gesture(self, coordinates):
        reshaped = np.asarray(coordinates.data).reshape(2,5,3)
        self.coor_diff = reshaped[0, :, :].reshape(5,3)
        self.coordinates = reshaped[1, :, :].reshape(5,3)
        gesture_observation = BF.gesture_observation(self.coor_diff)
        prediction = BF.prediction(gesture_observation)
        string_message = "Probabilty after gesture observation is: \n"
        for i in range(0, BF.speech.epsilon):
            string_message += "for " + BF.speech.objects[i].name + " is " + str(round(float(prediction[i]),3)) + "\n"

        rospy.loginfo(string_message)
        self.publisher_alexa.publish(string_message)

    def callback_speech(self, sentance):
        string_message = ""
        if sentance.data != "\n" and sentance.data != "":
            string_message += "Probabilty after speech observation is: \n"
            speech_observation = BF.speech_observation(self.coordinates, sentance.data)
            prediction = BF.prediction(speech_observation)
            for i in range(0, BF.speech.epsilon):
                string_message += "for " + BF.speech.objects[i].name + " is " + str(round(float(prediction[i]),3)) + "\n"
                
            rospy.loginfo(string_message)
            self.publisher_alexa.publish(string_message)


if __name__ == '__main__':
    rospy.init_node('bayes_listener', anonymous=True)
    BL = BayesListener()

    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        r.sleep()
