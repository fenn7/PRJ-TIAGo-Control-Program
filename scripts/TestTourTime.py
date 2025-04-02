#!/usr/bin/env python3
import os
import rospy
import actionlib
import time
from var5.msg import TourAction, TourGoal
from pal_interaction_msgs.msg import TtsActionGoal
from move_base_msgs.msg import MoveBaseActionGoal

# Test how long it takes to complete a lap
log_file = os.path.dirname(os.path.abspath(__file__)) + "/test/logs/TourTimeTestLog.txt"

def log_mb_goal(mb_goal):
    log(mb_goal)

def measure_time(start_time):
    final_time = time.time()
    elapsed = final_time - start_time
    log("One Lap Finished! Time: " + str(elapsed))

def log(input):
    rospy.loginfo(input)
    with open(log_file, 'a') as file:
        file.write(str(input) + '\n')

if __name__ == '__main__':
    try:
        rospy.init_node('client_tester', anonymous = True)
        client = actionlib.SimpleActionClient('tour_action', TourAction)
        client.wait_for_server()
        goal = TourGoal("quick")
        for i in range(5):
            start_time = time.time()
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(50.0))  
            measure_time(start_time)
        log("---------------------")    
    except rospy.ROSInterruptException:
        pass
