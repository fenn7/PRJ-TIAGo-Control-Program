#!/usr/bin/env python3
import rospy
import actionlib
import smach
import smach_ros
from PersonDetectorState import PersonDetectorState
from PersonDistanceState import PersonDistanceState
from NextFeatureNode import final_feature
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from var5.srv import GetNextFeatureRequest, GetNextFeature
from var5.msg import TourAction, TourResult
from smach import Concurrence, CBState
from smach_ros import SimpleActionState, ServiceState
from dynamic_reconfigure.client import Client

class TourServerNode:
    def __init__(self):
        print("Starting Tour Server")
        self.rate = rospy.Rate(5)
        self.nearest_person_data = (None, None)
        self.server = actionlib.SimpleActionServer('tour_action', TourAction, self.execute, False)
        self.teb_client = Client("/move_base/TebLocalPlannerROS", timeout=10, config_callback=self.modify_param_callback)
        self.slow_config_params = (0.15, 0.15, 0.0, 0.0)
        self.default_config_params = (0.5, 0.2, 2.0, 1.0)
        self.fast_config_params = (2.5, 1.5, 3.0, 2.0)
        self.current_move_goal = None
        self.stop_distance = 1.0 # 1.0 metre safe distance
        self.fast_distance = 5.0 # catch up to some person moving too far away (>5 metres).
        rospy.on_shutdown(self.shutdown_callback)

    def shutdown_callback(self):
        if (self.server.is_active()):
            rospy.loginfo("Server shut down by command-line input.")
            self.server.set_aborted()

    # Modifies parameters for an Timed-Elastic-Band Planner, based on the works of Roesmann et al. [1]
    def modify_param_callback(self, config):
        rospy.loginfo("Updated TEB planner configuration parameters.")

    def modify_params(self, config):
        # config is a 4-tuple of max_vel_x, acc_lim_x, weight_max_vel_x, weight_acc_lim_x
        params = {
            'max_vel_x': config[0],
            'acc_lim_x': config[1],
            'weight_max_vel_x': config[2],
            'weight_acc_lim_x': config[3]
        }
        self.teb_client.update_configuration(params)

    def should_preempt(self):
        return self.server.is_preempt_requested() or not self.server.is_active()

    # Start up and run the concurrency container as an action server.
    def execute(self, tour_goal):
        print("Goal Received. Starting new tour.")
        def termination_cb(outcome_map):
            if outcome_map['TOUR']:
                return True
            return False 

        concurrence = Concurrence(outcomes=['succeeded', 'preempted'], default_outcome='preempted',
                           outcome_map={'succeeded': {'TOUR': 'succeeded', 'TRACKER': 'succeeded', 'CONTROL': 'succeeded'}},
                           child_termination_cb = termination_cb)

        with concurrence:
            # The state machine that updates movement parameters, based on distance to the nearest person.
            # Find the current distance to the nearest person, and update parameters of the local TEB planner[1] accordingly.
            sm_nav_control = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
            with sm_nav_control:
                @smach.cb_interface(input_keys=['speed_mode'], output_keys=['speed_mode'], outcomes=['succeeded', 'preempted'])
                def reassess_param_cb(userdata, initial_speed):
                    userdata.speed_mode = initial_speed
                    if self.should_preempt():
                        return 'preempted'
                    nearest_person_distance = self.nearest_person_data[1] 
                    if nearest_person_distance is None:
                        if userdata.speed_mode != "mid":
                            userdata.speed_mode = "mid"
                            self.modify_params(self.slow_config_params)
                        return 'succeeded'
                    if nearest_person_distance < self.stop_distance and userdata.speed_mode != "slow":
                        userdata.speed_mode = "slow"
                        self.modify_params(self.slow_config_params)   
                    elif nearest_person_distance > self.fast_distance and userdata.speed_mode != "fast":
                        userdata.speed_mode = "fast"
                        self.modify_params(self.fast_config_params)
                    elif nearest_person_distance <= self.fast_distance and userdata.speed_mode != "mid":
                        userdata.speed_mode = "mid"
                        self.modify_params(self.default_config_params) 
                    return 'succeeded'
                smach.StateMachine.add('REASSESS_PARAMETERS', CBState(reassess_param_cb, cb_kwargs={'initial_speed' :'mid'}), {'succeeded': 'REASSESS_PARAMETERS',
                                                                                            'preempted': 'aborted'})
            Concurrence.add('CONTROL', sm_nav_control)

            # The state machine that detects where and who the nearest person is.
            sm_tracking = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
            with sm_tracking:
                # Get readings of tracked people
                smach.StateMachine.add('DETECT_PERSONS', PersonDetectorState(), transitions = {
                                                                            'people_detected': 'GET_DISTANCE_TO_NEAREST_PERSON',
                                                                            'preempted': 'aborted'},
                                                                            remapping = {'detections': 'detections'})
                # Calculate the distance to the nearest person, and store their ID.
                smach.StateMachine.add('GET_DISTANCE_TO_NEAREST_PERSON', PersonDistanceState(), transitions = {
                                                                            'nearest_distance_calculated': 'UPDATE_NEAREST',
                                                                            'preempted': 'aborted'},
                                                                            remapping = {'nearest_distance': 'distance', 'nearest_person': 'person'})
                # Update the distance data in the shared bufffer, so the CONTROL machine can use it.
                @smach.cb_interface(input_keys=['distance', 'person'], outcomes=['succeeded', 'preempted'])
                def update_nearest_cb(userdata):
                    if self.should_preempt():
                        return 'preempted'
                    self.nearest_person_data = (userdata.person, userdata.distance)
                    return 'succeeded'
                smach.StateMachine.add('UPDATE_NEAREST', CBState(update_nearest_cb), {'succeeded': 'DETECT_PERSONS',
                                                                                       'preempted': 'aborted'})
            Concurrence.add('TRACKER', sm_tracking)

            # The state machine that controls movement between points on a tour route.
            sm_moving = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])  
            with sm_moving:
                @smach.cb_interface(input_keys = ['feature_input'])
                def feature_request_cb(userdata, request):
                    feature_request = GetNextFeatureRequest()
                    feature_request.name = tour_goal.request
                    return feature_request
                
                # Get the next feature point to go to.
                smach.StateMachine.add('GET_POINT',
                                ServiceState('GetNextFeature', GetNextFeature,
                                request_cb = feature_request_cb,
                                input_keys = ['feature_input'],
                                response_slots = ['coord', 'explanation']),
                                transitions = {'succeeded': 'CHECK_COMPLETION', 'preempted': 'aborted'},
                                remapping = {'coord': 'target_pose', 'explanation' : 'speech'})

                # Check if a whole tour has been completed.
                @smach.cb_interface(input_keys=['target_pose', 'speech'], outcomes=['unfinished', 'finished', 'preempted'])
                def check_for_complete_cb(userdata):
                    if self.should_preempt():
                        return 'preempted'
                    if Point(userdata.target_pose.x, userdata.target_pose.y, userdata.target_pose.z) == final_feature[0]:
                        # an arbitrary final feature that signifies the end of a lap, and termination
                        rospy.loginfo("Tour Concluded! 1 Lap done.")
                        result = TourResult()
                        result.time = rospy.Time.now()
                        self.server.set_succeeded(result)
                        rospy.loginfo(result)
                        return 'finished'
                    return 'unfinished'

                smach.StateMachine.add('CHECK_COMPLETION', CBState(check_for_complete_cb), {'unfinished': 'GO_TO_POINT', 
                                                                                            'finished': 'succeeded', 
                                                                                            'preempted': 'aborted'})

                # move to the location of the current feature.
                def go_to_goal_cb(userdata, request):
                    move_base_goal = MoveBaseGoal()
                    move_base_goal.target_pose.header.stamp = rospy.get_rostime()
                    move_base_goal.target_pose.header.frame_id = 'map'
                    move_base_goal.target_pose.pose.position.x = userdata.target_pose.x
                    move_base_goal.target_pose.pose.position.y = userdata.target_pose.y
                    move_base_goal.target_pose.pose.orientation.w = 1
                    self.current_move_goal = move_base_goal
                    return move_base_goal
        
                smach.StateMachine.add('GO_TO_POINT',
                                SimpleActionState('/move_base', MoveBaseAction,
                                goal_cb = go_to_goal_cb,
                                input_keys = ['target_pose']),
                                transitions = {'succeeded': 'EXPLAIN_POINT', 'preempted': 'aborted'})

                # speak an explanation for the current feature
                def say_explanation_cb(userdata, request):
                    explanation = userdata.speech
                    rospy.loginfo(explanation)
                    tts_goal = TtsGoal()
                    tts_goal.rawtext.text = explanation
                    tts_goal.rawtext.lang_id = "en_GB"
                    return tts_goal

                smach.StateMachine.add('EXPLAIN_POINT',
                                SimpleActionState('/tts_to_soundplay', TtsAction,
                                goal_cb = say_explanation_cb,
                                input_keys = ['speech']),
                                transitions = {'succeeded': 'GET_POINT', 'preempted': 'aborted'})
                
            Concurrence.add('TOUR', sm_moving)
        outcome = concurrence.execute()

# start the main node and action server
if __name__ == '__main__':
    try:
        rospy.init_node('main_node', anonymous = True)
        server_node = TourServerNode()
        server_node.server.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#[1] Rösmann, Christoph & Feiten, Wendelin & Woesch, Thomas & Hoffmann, Frank & Bertram, Torsten. (2012). Trajectory modification considering dynamic constraints of autonomous robots. 1-6. 