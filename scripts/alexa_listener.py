#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from flask import Flask
from flask_ask import Ask, statement, question, session
import json
import requests
import time
import rospy

buffer = []
app = Flask(__name__)
ask = Ask(app, "/reddit_reader")

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    buffer.append("I see that " + data.data)

def listener():
    rospy.init_node('alexa_listener', anonymous=True)
    r = rospy.Rate(0.5)
    rospy.Subscriber("alexa_out", String, callback)
    r.sleep()

@app.route('/')
def homepage():
    return "Hi there, how ya doin?"

@ask.launch
def start_skill():
    welcome_message = 'Hi, can you direct the wand to the object?'
    return question(welcome_message)

@ask.intent("YesIntent")
def get_direction():
    listener()
    direction_msg = buffer.pop()
    return statement(direction_msg)

@ask.intent("NoIntent")
def no_intent():
    bye_text = 'I am not sure why you asked me to run then, but okay... bye'
    return statement(bye_text)

if __name__ == '__main__':
     app.run(host='0.0.0.0',port='8008')
