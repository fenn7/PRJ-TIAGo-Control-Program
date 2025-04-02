#!/usr/bin/env python3
import rospy
import smach
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class PersonDistanceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nearest_distance_calculated', 'preempted'],
                                    input_keys = ['detections'],
                                    output_keys = ['nearest_distance', 'nearest_person'])
        self.position_subscriber = rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, self.position_cb)
        self.position = None
    
    def position_cb(self, position):
        self.position = position.pose.pose.position

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if userdata.detections is not None and len(userdata.detections) > 0:
            distances_to_people = dict()
            for person in userdata.detections:
                person_position = person.pose.position
                # uses a euclidean as distance metric
                dist_x_to_person = self.position.x - person_position.x 
                dist_y_to_person = self.position.y - person_position.y
                euclidean_to_person = math.hypot(dist_x_to_person, dist_y_to_person)
                distances_to_people.update({person.id: euclidean_to_person})
            nearest_distance = min(distances_to_people.values())
            userdata.nearest_distance = nearest_distance
            nearest_person = min(distances_to_people, key=distances_to_people.get)
            userdata.nearest_person = nearest_person
            #print("nearest person id: " + str(nearest_person))
            #print("distance to them: " + str(nearest_distance))
        else:
            userdata.nearest_distance = None
            userdata.nearest_person = None
        return 'nearest_distance_calculated'
    
    def request_preempt(self):
         smach.State.request_preempt(self)
