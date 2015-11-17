#!/usr/bin/env python
#coding: utf-8

import sys
import time
import rospy
from std_msgs.msg import String, Empty, Float64

pub_withmeness = rospy.Publisher('withmeness_topic', Float64, queue_size=1)


task_targets = {
"WAITING_FOR_WORD": {"_/selection_tablet","_/experimentator"},
"RESPONDING_TO_NEW_WORD": {"_/robot_head","_/tablet","_/experimentator"},
"WAITING_FOR_LETTER_TO_FINISH": {"_/robot_head","_/tablet","_/experimentator"},
"ASKING_FOR_FEEDBACK": {"_/robot_head"},
"PUBLISHING_WORD": {"_/robot_head","_/selection_tablet","_/experimentator"},
"WAITING_FOR_FEEDBACK": {"_/tablet","_/experimentator","_/selection_tablet"},
"WAITING_FOR_ROBOT_TO_CONNECT": {"_/tablet","_/experimentator","_/robot_head"},
"WAITING_FOR_TABLET_TO_CONNECT": {"_/tablet","_/experimentator","_/robot_head"},
"RESPONDING_TO_DEMONSTRATION_FULL_WORD": {"_/robot_head","_/tablet","_/experimentator"},
"RESPONDING_TO_DEMONSTRATION": {"_/robot_head","_/tablet","_/experimentator"},
 }

current_task = "WAITING_FOR_WORD"
current_target = "_"
withmeness = 0.5
mu = 0.1

def onChangeTask(msg):
    global current_task
    current_task = (str)(msg.data)

def onChangeTarget(msg):
    global current_target
    current_target = (str)(msg.data)

if __name__=='__main__':

    global withmeness

    rospy.init_node("withmeness")

    while(True):
        # get current task:
        rospy.Subscriber("state_activity", String, onChangeTask)

        # get current target:
        rospy.Subscriber("actual_focus_of_attention", String, onChangeTarget)

        # compute EMA of online-with_me_ness:
        if current_task in task_targets:
            if current_target in task_targets[current_task]:
                withmeness = (1-mu)*withmeness + mu
            else:
                if current_target!="" and current_target!="_":
                    withmeness = (1-mu)*withmeness

        msg = Float64()
        msg.data = withmeness
        pub_withmeness.publish(msg)

        rospy.sleep(1.0)

    rospy.spin()
        


