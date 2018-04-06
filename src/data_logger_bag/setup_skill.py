#!/usr/bin/env python
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
from data_logger_bag.msg import LogControl
import intera_interface  # To use Sawyer SDK
import roslib
import rospy
from std_msgs.msg import Bool, String


class SkillSetup():
    NEUTRAL_JOINT_POS = [-0.9013076171875, -0.479220703125, -0.9098486328125, 0.1876142578125, 0.9019873046875, 1.960962890625, -0.912791015625]
    DEFAULT_ENV_OBJ_ARRAY = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
    DEFAULT_SETTINGS_NS = '/skill_settings'
    DEFAULT_SKILL_NAME = 'default_skill'

    def __init__(self):
        rospy.init_node('obj_pos_node', anonymous=True)

        default_c6_task_topic = "C6_Task_Description"
        default_log_flag = "data_logger_flag"

        self.limb = intera_interface.Limb('right')

        # Values for creating the publishers/subscribers
        self.c6_task_topic = rospy.get_param("~c6_task_topic", default_c6_task_topic)
        self.logger_flag_topic = rospy.get_param("~log_flag_topic", default_log_flag)
        self.logger_flag = False

        rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        rs.enable()
        rospy.loginfo('Enabled robot')

        self.env_obj_array = self.DEFAULT_ENV_OBJ_ARRAY
        self.init_joint_pos = self.NEUTRAL_JOINT_POS

        self.init_joints_msg = self.joint_angles_list_to_msg(self.init_joint_pos)
        self.env_obj_array_msg = self.posearray_list_to_msg(self.env_obj_array)

        self.skill = self.DEFAULT_SKILL_NAME
        self.runName = ""

        self.neutral_joint_pos_dict =  self.joint_angles_list_to_dict(self.NEUTRAL_JOINT_POS)

        # Check to make sure the user does want to move the robot to init
        response = raw_input("Do you want to move Sawyer back to its neutral position? Enter y/n \n")
        if response in ["y", "Y", 'yes', 'Yes']:
            self.limb.move_to_joint_positions(self.neutral_joint_pos_dict)  # Move Sawyer to a neutral position before starting
            rospy.sleep(1.0)
        
        self.obj_pub = rospy.Publisher("obj_pos_array", PoseArray, queue_size=10)  # Create new topic
        self.init_pub = rospy.Publisher("init_joint_pos", JointState, queue_size=10)

        rospy.Subscriber(self.logger_flag_topic, Bool, self.flag_callback)
        rospy.Subscriber(self.c6_task_topic, LogControl, self.change_log_settings_cb)

        if not rospy.has_param('/skill_settings'):
            rospy.logerr('Skill settings param not set! Make sure you load the yaml')

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.env_obj_array_msg is not None:  # Check if empty
                self.obj_pub.publish(self.env_obj_array_msg)
            if self.init_joints_msg is not None:
                self.init_pub.publish(self.init_joints_msg)
            rate.sleep()

        rospy.spin()

    '''
    Converts a list of joint values to dict of angles to be passed to intera_interface
    '''
    def joint_angles_list_to_dict(self, joint_angles_list):
        joint_angles_dict = self.limb.joint_angles()
        joint_angles_dict['right_j0'] = joint_angles_list[0]
        joint_angles_dict['right_j1'] = joint_angles_list[1]
        joint_angles_dict['right_j2'] = joint_angles_list[2]
        joint_angles_dict['right_j3'] = joint_angles_list[3]
        joint_angles_dict['right_j4'] = joint_angles_list[4]
        joint_angles_dict['right_j5'] = joint_angles_list[5]
        joint_angles_dict['right_j6'] = joint_angles_list[6]
        return joint_angles_dict

    '''
    Converts a list of joint values to JointState msg
    '''
    def joint_angles_list_to_msg(self, joint_angles_list):
        joint_angles_msg = JointState()
        joint_angles_msg.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        joint_angles_msg.position = joint_angles_list
        return joint_angles_msg


    '''
    Converts a vectorized object pose to Pose msg
    '''
    def pose_list_to_msg(self, pose_list):
        pose_msg = Pose()
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        pose_msg.orientation.x = pose_list[3]
        pose_msg.orientation.y = pose_list[4]
        pose_msg.orientation.z = pose_list[5]
        pose_msg.orientation.w = pose_list[6]
        return pose_msg


    '''
    Converts a list of vectorized object poses to PoseArray msg
    '''
    def posearray_list_to_msg(self, posearray_list):
        posearray_msg = PoseArray()
        for pose_list in posearray_list:
            pose_msg = self.pose_list_to_msg(pose_list)
            posearray_msg.poses.append(pose_msg)
        return posearray_msg


    '''
    Populate the PoseArray obj_poses with the positions of each object
        depending on the currently-executing skill and trial # 
    '''
    def change_skill_settings(self, skill_name, trial_name):
        trial_name_splitted = trial_name.split("-")
        init_name = trial_name_splitted[0]
        env_name = trial_name_splitted[1]
        str_info = 'Setting Skill = ' + str(skill_name) + ', INIT = ' + str(init_name), ', ENV = ' + str(env_name)

        rospy.loginfo(str_info)

        skill_param_name = self.DEFAULT_SETTINGS_NS + '/' + skill_name.lower()
        init_param_name = skill_param_name + '/init/' + init_name[-1]
        env_param_name = skill_param_name + '/env/' + env_name[-1]

        if rospy.has_param(skill_param_name):
            self.init_joint_pos = rospy.get_param(init_param_name, self.NEUTRAL_JOINT_POS)
            self.env_obj_array = rospy.get_param(env_param_name, self.DEFAULT_ENV_OBJ_ARRAY)

            if not rospy.has_param(init_param_name):
                rospy.logwarn('Init param does not exist. DEFAULTING!')
            if not rospy.has_param(env_param_name):
                rospy.logwarn('Env param does not exist. DEFAULTING!')
        else:
            rospy.logerr('Skill does not exist. NO CHANGE!')

        self.init_joints_msg = self.joint_angles_list_to_msg(self.init_joint_pos)
        self.env_obj_array_msg = self.posearray_list_to_msg(self.env_obj_array)


    '''
    Update the skill and trial when changed and move the robot to specified init position
    '''
    def change_log_settings_cb(self, msg):
        if self.logger_flag is False:

            if msg.skillName is not "":
                self.skill = msg.skillName

            if msg.runName is not "":
                self.runName = msg.runName

            self.change_skill_settings(self.skill, self.runName)

            # Check to make sure the user does want to move the robot to init
            response = raw_input("Do you want to move Sawyer to the initial position? Enter y/n \n")
            if response in ["y", "Y", 'yes', 'Yes']:
                init_joints_dict = self.joint_angles_list_to_dict(self.init_joint_pos)
                self.limb.move_to_joint_positions(init_joints_dict)
                rospy.sleep(0.5)

            rospy.loginfo("You can start recording!")
        else:
            rospy.loginfo("Currently still writing previous record. Settings for logging NOT changed")


    '''
    Callback for when to trigger recording of bag files
    '''
    def flag_callback(self, msg):

        # Setup flag from message
        rospy.loginfo(rospy.get_name() + ": I heard %s" % msg.data) 
       
        # Checks for change in flag first
        if self.logger_flag != msg.data:
            
            rospy.loginfo("Detected flag state change")

            # Set the flag after checking 
            self.logger_flag = msg.data

            # Then check what the message is telling us
            if msg.data is True:
                self.logger_flag = True
            else:
                self.logger_flag = False



if __name__ == '__main__':
    SkillSetup()
    rospy.spin()  # Prevents python from exiting until node is stopped
