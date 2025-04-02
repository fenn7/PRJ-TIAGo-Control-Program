#!/usr/bin/env python3
from tokenize import String
import rospy
import random
from geometry_msgs.msg import Point
from var5.srv import GetNextFeature, GetNextFeatureResponse

# can define your own custom tours here
sample_tour = ("reemc", 
               (
                (Point(4.5, -11, 0), "This is Feature 1"), 
               (Point(-7.5, -12.5, 0), "This is Feature 2"),
               (Point(0, -7.5, 0), "This is Feature 3"),
               (Point(0, 0, 0), "Back at the start again!")
               )
               )
quick_test = ("quick", 
         (
        (Point(5, 0, 0), "1"),
         (Point(0, 0, 0), "0")
         )
         )

global final_feature
final_feature = (Point(-99999, -99999, -99999), "") # this can be anything, but recommended to be an infeasible navigation point

class NextFeatureNode():
    def __init__(self, identified_tours):
        # setup defined tours as a map
        self.tours = {}
        for tour in identified_tours:
            self.tours.update({tour[0]: tour[1]})
        # tracks number of times service is called
        self.service_calls_total = {}

    def get_next_feature(self, request):
        # ID of the tour we wish to carry out.
        tour_identifier = request.name
        if not tour_identifier in self.service_calls_total.keys():
            # if a tour route has not previously been run, add it here.
            self.service_calls_total.update({tour_identifier : 0})
        
        # if there are still available features to visit in the tour, get the next one, else return the final feature.
        # uses dictionary lookup
        if self.service_calls_total[tour_identifier] < len(self.tours.get(tour_identifier)):
            next_feature = self.tours[tour_identifier][self.service_calls_total[tour_identifier]]
            next_feature_response = GetNextFeatureResponse(next_feature[0], next_feature[1])
            self.service_calls_total[tour_identifier] += 1
        else:
            next_feature_response = final_feature
            self.service_calls_total[tour_identifier] = 0
        # rospy.loginfo(str(next_feature_response))
        return next_feature_response

    def next_feature_service(self):
        rospy.init_node('next_feature_node', anonymous=True)
        srv = rospy.Service('GetNextFeature', GetNextFeature, self.get_next_feature)

if __name__ == "__main__":
    try:
        # add custom tours to this variable to allow the service to define them at runtime
        tours = (sample_tour, quick_test)
        next_feature_node = NextFeatureNode(tours)
        next_feature_node.next_feature_service()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
