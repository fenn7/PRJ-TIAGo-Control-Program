#!/usr/bin/env python3
import rospy
import os
import actionlib
from var5.msg import TourAction, TourGoal
from pal_interaction_msgs.msg import TtsActionGoal
from move_base_msgs.msg import MoveBaseActionGoal

# Test that the navigation function of the system works.
log_file = os.path.dirname(os.path.abspath(__file__)) + "/test/logs/NavigationTestLog.txt"

def log_mb_goal(mb_goal):
    log(mb_goal)

def log_tts_goal(tts_goal):
    log(tts_goal)

def log(input):
    rospy.loginfo(input)
    with open(log_file, 'a') as file:
        file.write(str(input) + '\n')

if __name__ == '__main__':
    print((log_file))
    try:
        rospy.init_node('client_tester', anonymous = True)
        # need to send a goal to start system.
        client = actionlib.SimpleActionClient('tour_action', TourAction)
        movebase_cb = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, log_mb_goal)
        tts_cb = rospy.Subscriber('/tts_to_soundplay/goal', TtsActionGoal, log_tts_goal)
        client.wait_for_server()

        log("STARTING NEW NAVIGATION TEST")
        log("----------------------------")

        # repetition
        goal = TourGoal("quick")
        for i in range(4):
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(50.0))
            log(client.get_result())

        goal = TourGoal("reemc")
        for i in range(2):
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(50.0))
            log(client.get_result())
    except rospy.ROSInterruptException:
        pass



