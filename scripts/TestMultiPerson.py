#!/usr/bin/env python3
import os
import rospy
import time
from PersonPublishers import FixedPersonPublisher

# Test that the navigation function of the system works.
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
        rospy.init_node('multi_person_simulator')
        publisher = FixedPersonPublisher([(0, 0), (6, 0), (-4, -1.5)])
        publisher.publish()
    except rospy.ROSInterruptException:
        pass
