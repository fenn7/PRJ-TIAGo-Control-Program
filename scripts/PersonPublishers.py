#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from leg_tracker.msg import PersonArray, Person
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

def create_person_and_marker(x, y, id):
    marker = Marker()
    marker.header.stamp = rospy.Time()
    marker.header.frame_id = "map"
    marker.ns = "test_namespace"
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.5
    marker.id = id

    person = Person()
    person.pose.position.x = x
    person.pose.position.y = y
    person.pose.position.z = 0
    person.id = id
    return (person, marker)

# simulates a person existing at a certain fixed point. 
# For when the robot is running in simulation and cannot access the leg_tracker package
class FixedPersonPublisher:
    def __init__(self, person_positions):
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)
        self.tracked_people_pub = rospy.Publisher('people_tracked', PersonArray, queue_size=100)
        self.pose_tracker = rospy.sub = rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, self.pose_cb)
        self.person_positions = person_positions
        if self.person_positions is None:
            try:
                launch_x = rospy.get_param('launch_pose_x')
                launch_y = rospy.get_param('launch_pose_y')
                rospy.loginfo(self.person_positions)
                self.person_positions = [(int(launch_x), int(launch_y))]
            except ValueError:
                pass
        self.pose = (0, 0)
        self.id = 0

    def pose_cb(self, pose_data):
        position = pose_data.pose.pose.position
        self.pose = (position.x, position.y)

    def publish(self):
        markers = MarkerArray()
        persons = PersonArray()
        for position in self.person_positions:
            (person, marker) = create_person_and_marker(position[0], position[1], self.id)
            persons.people.append(person)
            markers.markers.append(marker)
            self.id += 1
        while not rospy.is_shutdown():
            self.marker_pub.publish(markers)
            self.tracked_people_pub.publish(persons)

if __name__ == '__main__':
     try:
         rospy.init_node('person_simulator')
         publisher = FixedPersonPublisher([(0, 0)])
         publisher.publish()
     except rospy.ROSInterruptException:
         pass
