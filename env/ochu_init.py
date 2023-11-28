import subprocess
import rospkg
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import numpy as np
import math
import tf.transformations as tf_trans

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryFeedback

from gazebo_msgs.srv import GetModelState

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
## roslaunch
def launch_file(ros_package, launch_file_name):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path(ros_package)
    launch = launch_file_name
    package_path += "/launch/" + launch
    print(package_path)
    subprocess.Popen(["roslaunch", package_path])

#launch_file('ur10_ochu_moviet', "demo_gazebo.launch")

## moviet planning
#!/usr/bin/env python
def run_file(ros_package, run_file_name):
    command = ['rosrun', ros_package, run_file_name]
    subprocess.run(command)



def move_in_effector_sight(xm, ym, zm, xr, yr, zr, theta):
    theta = math.pi*theta/180
    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.init_node('move_in_effector_sight', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")
    #gripper_group = moveit_commander.MoveGroupCommander("gripper")
    pose = group.get_current_pose()
    print(pose.pose)
    pose_target = geometry_msgs.msg.Pose()
    px = pose.pose.position.x
    py = pose.pose.position.y
    pz = pose.pose.position.z
    x = pose.pose.orientation.x
    y = pose.pose.orientation.y
    z = pose.pose.orientation.z
    w = pose.pose.orientation.w
    transform_matrix = np.array([
    [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w, px],
    [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w, py],
    [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2, pz],
    [0, 0, 0, 1]
    ])
    vector = [xm,ym,zm,1]

    target_position = np.matmul(transform_matrix, vector)

    vector = [xr,yr,zr,1]
    target_orientation = np.matmul(transform_matrix, vector)-[px, py, pz, 0]
    pose_target.orientation.w = w * math.cos(theta/2) - x * target_orientation[0]*math.sin(theta/2) - y * target_orientation[1]*math.sin(theta/2) - z * target_orientation[2]*math.sin(theta/2)
    pose_target.orientation.x = w * target_orientation[0]*math.sin(theta/2) + x * math.cos(theta/2) + y * target_orientation[2]*math.sin(theta/2) - z * target_orientation[1]*math.sin(theta/2)
    pose_target.orientation.y = w * target_orientation[1]*math.sin(theta/2) + y * math.cos(theta/2) + z * target_orientation[0]*math.sin(theta/2) - x * target_orientation[2]*math.sin(theta/2)
    pose_target.orientation.z = w * target_orientation[2]*math.sin(theta/2) + z * math.cos(theta/2) + x * target_orientation[1]*math.sin(theta/2) - y * target_orientation[0]*math.sin(theta/2)
    # pose_target.orientation.w = math.cos(theta/2)
    # pose_target.orientation.x = target_orientation[0]*math.sin(theta/2)
    # pose_target.orientation.y = target_orientation[1]*math.sin(theta/2)
    # pose_target.orientation.z = target_orientation[2]*math.sin(theta/2)
    pose_target.position.x = target_position[0] 
    pose_target.position.y = target_position[1]  
    pose_target.position.z = target_position[2] 
    group.set_pose_target(pose_target)
    print(pose_target)
    plan = group.plan()
    group.stop()
    group.clear_pose_targets()
    group_names = robot.get_group_names()
    print("Available Planning Groups:", group_names)
    
    s = plan[0]

    if s:   
        rospy.loginfo("Motion plan executed successfully")
        group.execute(plan[1], wait=True)
        
    else:
        rospy.loginfo("Motion plan execution failed")
    moveit_commander.roscpp_shutdown()
    return s

def move_in_human_sight(xm, ym, zm, xr, yr, zr, theta):
    theta = math.pi*theta/180
    moveit_commander.roscpp_initialize(sys.argv)
    
    # rospy.init_node('move_in_effector_sight', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")
    #gripper_group = moveit_commander.MoveGroupCommander("gripper")
    pose = group.get_current_pose()
    print(pose.pose)
    pose_target = geometry_msgs.msg.Pose()
    px = pose.pose.position.x
    py = pose.pose.position.y
    pz = pose.pose.position.z
    x = pose.pose.orientation.x
    y = pose.pose.orientation.y
    z = pose.pose.orientation.z
    w = pose.pose.orientation.w
    vector = [xm,ym,zm,1]
    #target_position = vector 
    target_position = [px+xm,py+ym,pz+zm,1]
    roll = np.radians(xr)   # 转换为弧度
    pitch = np.radians(yr)
    yaw = np.radians(zr)
    q = tf_trans.quaternion_from_euler(roll, pitch, yaw)
    vector = [xr,yr,zr,theta]
    target_orientation = vector 
    # pose_target.orientation.w = w * math.cos(theta/2) - x * target_orientation[0]*math.sin(theta/2) - y * target_orientation[1]*math.sin(theta/2) - z * target_orientation[2]*math.sin(theta/2)
    # pose_target.orientation.x = w * target_orientation[0]*math.sin(theta/2) + x * math.cos(theta/2) + y * target_orientation[2]*math.sin(theta/2) - z * target_orientation[1]*math.sin(theta/2)
    # pose_target.orientation.y = w * target_orientation[1]*math.sin(theta/2) + y * math.cos(theta/2) + z * target_orientation[0]*math.sin(theta/2) - x * target_orientation[2]*math.sin(theta/2)
    # pose_target.orientation.z = w * target_orientation[2]*math.sin(theta/2) + z * math.cos(theta/2) + x * target_orientation[1]*math.sin(theta/2) - y * target_orientation[0]*math.sin(theta/2)
    # pose_target.orientation.w = math.cos(theta/2)
    # pose_target.orientation.x = target_orientation[0]*math.sin(theta/2)
    # pose_target.orientation.y = target_orientation[1]*math.sin(theta/2)
    # pose_target.orientation.z = target_orientation[2]*math.sin(theta/2)
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]
    pose_target.position.x = target_position[0]
    pose_target.position.y = target_position[1]
    pose_target.position.z = target_position[2]
    group.set_pose_target(pose_target)
    print(pose_target)
    plan = group.plan()
    group.stop()
    group.clear_pose_targets()
    group_names = robot.get_group_names()
    print("Available Planning Groups:", group_names)
    
    s = plan[0]

    if s:   
        rospy.loginfo("Motion plan executed successfully")
        group.execute(plan[1], wait=True)
        
    else:
        rospy.loginfo("Motion plan execution failed")
    moveit_commander.roscpp_shutdown()
    return s, 

# def get_arm_pose():

#time.sleep(10000)
#rospy.init_node('move_in_effector_sight', anonymous=True)
#move_in_effector_sight(0,0,-0.1, 0, 0, 1, 45) # xm ym zm xr yr zr theta

# move_in_human_sight(0,0,-0.1, 30, 0, 0, 0) # xm ym zm xr yr zr theta

def hand_joint_grasping_command(a1, a2, a3, b1, b2, c1, c2, d1, d2, e1, e2):
    pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=100)
    # rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10)  # 10hz
    trajectory = JointTrajectory()
    trajectory.joint_names = [
        'hand_base_to_finger1_rotate_axis',
        'finger1_rotate_axis_to_finger1',
        'finger1_to_finger1_1',
        'hand_base_to_finger2',
        'finger2_to_finger2_1',
        'hand_base_to_finger3',
        'finger3_to_finger3_1',
        'hand_base_to_finger4',
        'finger4_to_finger4_1',
        'hand_base_to_finger5',
        'finger5_to_finger5_1'
    ]
    point = JointTrajectoryPoint()
    point.positions = [a1, a2, a3, b1, b2, c1, c2, d1, d2, e1, e2]
    # point.effort = [a1, a2, a3, b1, b2, c1, c2, d1, d2, e1, e2]
    point.time_from_start = rospy.Duration(1)
    trajectory.points.append(point)
    # 
    pub.publish(trajectory)


# rospy.init_node('talker', anonymous=True)
# elapsed_time = rospy.Time.now()
# start_ros_time = rospy.Time.now()
# period = rospy.Duration(1./10)
# i = 0

# while True:
#     hand_joint_grasping_command(1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5)
#     # hand_joint_grasping_command(0,0,0,0,0,0,0,0,0,0,0)
#     # elapsed_time = rospy.Time.now() - start_ros_time
#     # if elapsed_time > period*(4.0/5):
#     #     if elapsed_time > period:
#     #         break
#     #     else:
#     #         rospy.sleep(period-elapsed_time)
#     #         break
#     # else:
#     #     rospy.sleep(period/5.0)
#     if i > 3 :
#         i = 0
#         break
#     i += 1
#     rospy.sleep(period)

# object_position = lambda : rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("coke_can", "").pose.position
# print(object_position())
def _distance_to_target(link, target):
    return np.sqrt(np.sum([(x-y)**2 for x, y in zip(link, target)]))

def state_callback(data):
    # 这里处理接收到的消息
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.actual.positions)

def hand_joint_state():
    # rospy.init_node('gripper_state_listener', anonymous=True)
    # rospy.Subscriber("/gripper_controller/state", FollowJointTrajectoryFeedback, state_callback, queue_size=1)
    data = rospy.wait_for_message("/gripper_controller/state", FollowJointTrajectoryFeedback, 10)
    # state_callback(data)
    return data

# hand_joint_state()
# rospy.init_node('gripper_state_listener', anonymous=True)
# state_callback(hand_joint_state())

# middle_finger_position = lambda : rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("robot", "finger3_1").pose.position
# print(middle_finger_position())

# rospy.init_node('gripper_state_listener', anonymous=True)
# raw_image = rospy.wait_for_message("/camera/depth/image_raw", Image, 10.0)
# image = CvBridge().imgmsg_to_cv2(raw_image, "32FC1")
# print(image)