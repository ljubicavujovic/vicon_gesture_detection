#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from scipy.stats import multivariate_normal
from bayes_filter import BayesFilter
from speech import Speech
from sys import stdin

c = 0.8
transition_model = np.zeros((5, 5))
transition_model.fill(1.*(1-c)/(transition_model.shape[0]-1))
np.fill_diagonal(transition_model, c)
BF = BayesFilter(transition_model)

back_up_sentance = "Fetch pick this object"

def callback(coordinates):
    reshaped = np.asarray(coordinates.data).reshape(2,5,3)
    coor_diff = reshaped[0, :, :].reshape(5,3)
    coordinates = reshaped[1, :, :].reshape(5,3)

    print "Type the sentance : "
    sentance = stdin.readline()
    if sentance != "\n":
        speech_observation = BF.speech_observation(coordinates, sentance)
        prediction = BF.prediction(speech_observation)

        for i in range(0, 5):
            rospy.loginfo("Probabilty after speech observation for " +
            BF.speech.objects[i] + " is " + str(prediction[i]) + "\n")

    gesture_observation = BF.gesture_observation(coor_diff)
    prediction = BF.prediction(gesture_observation)
    for i in range(0, 5):
        rospy.loginfo("Probabilty after gesture observation for " +
        BF.speech.objects[i] + " is " + str(prediction[i]) + "\n")


if __name__ == '__main__':
    rospy.init_node('bayes_listener', anonymous=True)
    rospy.Subscriber("coordinates", Float64MultiArray, callback)
    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        r.sleep()
