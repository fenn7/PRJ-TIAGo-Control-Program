#!/usr/bin/env python3
import rospy
import actionlib
from var5.msg import TourAction, TourGoal

# send a goal, the id of a tour, to the TourServer
if __name__ == '__main__':
    try:
        rospy.init_node('client_main', anonymous = True)
        client = actionlib.SimpleActionClient('tour_action', TourAction)
        client.wait_for_server()
        goal = TourGoal("quick") ## if you defined a custom tour, replace this with your own.
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(5.0))
    except rospy.ROSInterruptException:
        pass
