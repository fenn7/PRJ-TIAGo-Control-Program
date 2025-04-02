#!/usr/bin/env python3
import os
import smach
import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

r = rospkg.RosPack()
reemc_test_walls = os.path.join(r.get_path('pal_gazebo_worlds'), 'models', 'reemc_walls', 'walls.sdf')

# purpose of this class is to fully set up the gazebo simulation such that it matches the reemc_walls rviz test world.
def setup_simulation():
    rospy.init_node('gazebo_simulation_setup', anonymous=True)
    srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    file = open(reemc_test_walls)
    sdf_model = file.read()
    reemc_pose = Pose()
    reemc_pose.position.x = -7
    reemc_pose.position.y = -6
    tiago_state = ModelState()
    tiago_state.model_name = "tiago"
    tiago_state.pose.orientation.w = 1
    tiago_state.reference_frame = "map"
    new_model = srv("var5_reemc_test", sdf_model, "tiago_socialnav_sim", reemc_pose, "map")
    model_pub.publish(tiago_state)
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == "__main__":
    try:
        setup_simulation()
    except rospy.ROSInterruptException:
        pass