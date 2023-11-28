from ochu_init import launch_file
from ochu_init import move_in_human_sight
from ochu_init import hand_joint_grasping_command
from ochu_init import _distance_to_target
from ochu_init import run_file
from ochu_init import hand_joint_state

import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelConfiguration
import os, time
import numpy as np
from cv_bridge import CvBridge
import gym

class grasp_the_target():
    """
    """
    def __init__(self, frequency):
        # launch_path = "/".join(os.path.realpath(__file__).split("/")[:-2] + ["launch", "demo_gazebo.launch"])
        # launch_file('ur10_ochu_moviet', "demo_gazebo.launch")

        self.action_dimension = 17 # gripper: 11; manipulator: 6
        self.observation_dimension = ((160, 160), 44)
        self.object_position = lambda : rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("coke_can", "").pose.position # target position
        self.middle_finger_position = lambda : rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("robot", "finger3_1").pose.position
        self.target = self.object_position()
        self.gripper_initial_joint_angle = np.zeros(11, dtype=np.float32)

        #Task parameter.          # UNIT
        self.no_collision_happen = 5.0 # reward
        self.reach_goal = 1.5     # reward
        self.pick_goal = 5.0      # reward        # """ ##############################r1 = self.reach_goal if  distance < self.limit_distance else np.exp(-distance)
        # r2 = self.pick_goal if z > 0.1 else 0.0##
        #     ## CC ##########################
        #     ## CC ##++++++++++++############
        #     ########++++++++++++#### UR10 ##
        #     ########++++++++++++#### UR10 ##
        #     ########++++++++++++#### UR10 ##
        #     ########++++++++++++#### UR10 ##
        #     ########++++++++++++############
        #     ################################
        #     ################################
        #     Position of the object is randomly chosen from a uniform distribution.
        #     Object can be spawn anywhere on the '+' area.
        # """r1 = self.reach_goal if  distance < self.limit_distance else np.exp(-distance)
        # r2 = self.pick_goal if z > 0.1 else 0.0
        rospy.init_node("ur_10_ochu_reinforce_node", anonymous=True)

        # self.reset_hand = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
        self.frequency = frequency
        self.period = rospy.Duration(1./frequency)

    # def _camera_callback(self, data):
    #     self.serialized_image = data

    # def _hand_callback(self, data):
    #     self.joint_state = data.actual.positions

    def step(self, action):
        xm, ym, zm, xr, yr, zr, a1, a2, a3, b1, b2, c1, c2, d1, d2, e1, e2 = action ## a1: (0.022, 1.10) a2: (0.047, 0.658) a3~e2: (0,1.5) xm~zm: (-0.1, 0.1) xr~ zr: (-180, 180)

        ## action start
        while True:
            hand_joint_grasping_command(a1, a2, a3, b1, b2, c1, c2, d1, d2, e1, e2)
            if i > 3 :
                i = 0
                break
            i += 1
            rospy.sleep(self.period)
        s = move_in_human_sight(xm, ym, zm, xr, yr, zr, 0)

        ## observation
        # self.camera_subscriber = rospy.Subscriber("/rrbot/camera/depth/image_raw", Image, callback=self._camera_callback, queue_size=2)
        # self.hand_subscriber = rospy.Subscriber("/gripper_controller/state", FollowJointTrajectoryFeedback, state_callback, queue_size=1)
        raw_image = rospy.wait_for_message("/camera/depth/image_raw", Image, 10.0)
        self.serialized_image = raw_image
        self.joint_states = hand_joint_state()
        self.m_position = self.middle_finger_position()
        distance = _distance_to_target(self.m_position , self.target)
        if(s == 0):
            collision = 1
            reward = self._reward(distance, collision)
        # image = CvBridge().imgmsg_to_cv2(self.serialized_image, "rgb8")
        image = CvBridge().imgmsg_to_cv2(self.serialized_image, "32FC1")
        return ((image, self.joint_states), reward, False, {})

    def reset_robot(self):
        i = 0
        while True:
            hand_joint_grasping_command(0,0,0,0,0,0,0,0,0,0,0)
            if i > 3 :
                i = 0
                break
            i += 1
            rospy.sleep(self.period)
        run_file('arm_planning',"motion_command2")
    
    def reset(self):

        self.set_random_object_position("coke_can")
        self.reset_robot()
        joint_states = hand_joint_state()
        raw_image = rospy.wait_for_message("/camera/depth/image_raw", Image, 10.0)
        print ("--------------------------------------------------------")
        # self.pause()
        image = CvBridge().imgmsg_to_cv2(raw_image, "32FC1")
        return (image, joint_states)



    def set_random_object_position(self, name):
        # """ ##############################r1 = self.reach_goal if  distance < self.limit_distance else np.exp(-distance)
        # r2 = self.pick_goal if z > 0.1 else 0.0##
        #     ## CC ##########################
        #     ## CC ##++++++++++++############
        #     ########++++++++++++#### UR10 ##
        #     ########++++++++++++#### UR10 ##
        #     ########++++++++++++#### UR10 ##
        #     ########++++++++++++#### UR10 ##
        #     ########++++++++++++############
        #     ################################
        #     ################################
        #     Position of the object is randomly chosen from a uniform distribution.
        #     Object can be spawn anywhere on the '+' area.
        # """r1 = self.reach_goal if  distance < self.limit_distance else np.exp(-distance)
        # r2 = self.pick_goal if z > 0.1 else 0.0
        set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        model_state = ModelState()
        model_state.model_name = name
        model_state.pose.position.x = np.random.uniform(low=-0.7, high=0.5)
        model_state.pose.position.y = np.random.uniform(low=-0.39, high=1.1 )
        model_state.pose.position.z = 0
        model_state.reference_frame = "world"
        set_model(model_state)

    def _reward(self, distance, collision):
        if collision:
            r1 = self.no_collision_happen
        else:
            r1 = 0
        if distance < 0.05:
            r2 = self.reach_goal
        else:
            r2 = 0
        return r1 + r2
