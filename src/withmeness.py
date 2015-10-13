#!/usr/bin/env python
#coding: utf-8

import sys
import time
import rospy
from std_msgs.msg import String, Empty, Float64

pub_withmeness = rospy.Publisher('withmeness_topic', Float64, queue_size=1)


task_targets = {
"WAITING_FOR_WORD": {"selection_tablet","experimentator"},
"RESPONDING_TO_NEW_WORD": {"robot_head","tablet","experimentator"},
"WAITING_FOR_LETTER_TO_FINISH": {"robot_head","tablet","experimentator"},
"ASKING_FOR_FEEDBACK": {"robot_head"},
"PUBLISHING_WORD": {"robot_head","selection_tablet","experimentator"},
"WAITING_FOR_FEEDBACK": {"tablet","experimentator","selection_tablet"},
"WAITING_FOR_ROBOT_TO_CONNECT": {"tablet","experimentator","robot_head"},
"WAITING_FOR_TABLET_TO_CONNECT": {"tablet","experimentator","robot_head"},
"RESPONDING_TO_DEMONSTRATION_FULL_WORD": {"robot_head","tablet","experimentator"},
"RESPONDING_TO_DEMONSTRATION": {"robot_head","tablet","experimentator"},
 }

current_task = ""
current_target = ""
withmeness = 0.5
mu = 0.1

def onChangeTask(msg):
    global current_task
    current_task = msg.data

def onChangeTarget(msg):
    global current_target
    current_target = msg.data

if __name__=='__main__':

    rospy.init_node("withmeness")

    # get current task:
    rospy.Subscriber("state_activity", String, onChangeTask)

    # get current target:
    rospy.Subscriber("actual_focus_of_attention", String, onChangeTarget)

    # compute EMA of online-with_me_ness:
    if current_task in task_targets:
        if current_target in task_targets[task]:
            withmeness = (1-mu)*withmeness + mu
        else:
            if current_target!=" ":
                withmeness = (1-mu)*withmeness

    # publish with_me_ness on 'withmeness_topic'
    msg = Float64()
    msg.data = withmeness
    pub_withmeness.publish(msg)

    rospy.sleep(1.0)

    rospy.spin()
        


