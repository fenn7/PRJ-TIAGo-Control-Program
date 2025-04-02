#!/usr/bin/env python3
import rospy
import smach
from leg_tracker.msg import PersonArray

class PersonDetectorState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['people_detected', 'preempted'],
                                    output_keys = ['detections'])
        self.detector = rospy.Subscriber('people_tracked', PersonArray, self.person_callback)
        self.detections = None

    def person_callback(self, person_array):
        self.detections = person_array

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if self.detections is None:
            userdata.detections = None
        else:
            userdata.detections = self.detections.people
            self.detections = None
        return 'people_detected'
    
    def request_preempt(self):
         smach.State.request_preempt(self)
