#!/usr/bin/env python
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
from data_logger_bag.msg import LogControl
import intera_interface  # To use Sawyer SDK
import roslib
import rospy
from std_msgs.msg import Bool, String


class ObjPose():

    def __init__(self):
        rospy.init_node('obj_pos_node', anonymous=True)

        default_c6_task_topic = "C6_Task_Description"
        default_log_flag = "data_logger_flag"

        self.limb = intera_interface.Limb('right')

        self.obj_poses = None
        self.init_joints = None
        self.robot_init_pos = self.limb.joint_angles()
        self.trial_init_pos = self.limb.joint_angles()  # []

        self.neutral_joint_pos = self.get_neutral_joint_pos()

        self.skill = "default_skill"
        self.runName = ""

        # Values for creating the publishers/subscribers
        self.c6_task_topic = rospy.get_param("~c6_task_topic", default_c6_task_topic)
        self.logger_flag_topic = rospy.get_param("~log_flag_topic", default_log_flag)
        self.logger_flag = False

        # Check to make sure the user does want to move the robot to init
        response = raw_input("Do you want to move Sawyer back to its neutral position? Enter y/n \n")
        if response in ["y", "Y", 'yes', 'Yes']:
            self.limb.move_to_joint_positions(self.neutral_joint_pos)  # Move Sawyer to a neutral position before starting
            rospy.sleep(0.5)
        
        self.obj_pub = rospy.Publisher("obj_pos", PoseArray, queue_size=10)  # Create new topic
        self.init_pub = rospy.Publisher("init_joint_pos", JointState, queue_size=10)

        rospy.Subscriber(self.logger_flag_topic, Bool, self.flag_callback)
        rospy.Subscriber(self.c6_task_topic, LogControl, self.change_log_settings_cb)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.obj_poses is not None:  # Check if empty
                self.obj_pub.publish(self.obj_poses)
            if self.init_joints is not None:
                self.init_pub.publish(self.init_joints)
            rate.sleep()

        rospy.spin()

    '''
    Move Sawyer back to our specific initial position
    '''
    # [0.4677822265625, -0.9013076171875, -0.479220703125, -0.9098486328125, 0.1876142578125, 0.9019873046875, 1.960962890625, -0.912791015625, 0.0]
    def get_neutral_joint_pos(self):
        neutral_joint_pos = self.limb.joint_angles()
        neutral_joint_pos['right_j0'] = -0.9013076171875
        neutral_joint_pos['right_j1'] = -0.479220703125
        neutral_joint_pos['right_j2'] = -0.9098486328125
        neutral_joint_pos['right_j3'] = 0.1876142578125
        neutral_joint_pos['right_j4'] = 0.9019873046875
        neutral_joint_pos['right_j5'] = 1.960962890625
        neutral_joint_pos['right_j6'] = -0.912791015625
        return neutral_joint_pos

    '''
    Populate the PoseArray obj_poses with the positions of each object
        depending on the currently-executing skill and trial # 
    '''
    def change_env(self, skill, trial):
        print('change_env executing')
        self.obj_poses = PoseArray()  # Clear obj_poses each time
        print skill

        if skill == 'REACHING':
            self.init_joints = JointState()
            self.init_joints.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
            self.init_joints.position =   [-0.9013076171875, -0.479220703125, -0.9098486328125, 0.1876142578125, 0.9019873046875, 1.960962890625, -0.912791015625]
            
            if trial == 'RUN0':
                pose = Pose()
                pose.position.x = 1
                pose.position.y = 2
                pose.position.z = 3
                self.obj_poses.poses.append(pose)
            else:
                rospy.logerr('No environment info provided for this trial')
        else:
            rospy.logerr('Skill does not exist!')


        # if skill is 'REACHING':
        #     # TODO: hard code these based on the value combination
        #     if trial is 'RUN0':
        #     elif trial is 'RUN1':
        #     elif trial is 'RUN2':
        #     elif trial is 'RUN3':
        #     elif trial is 'RUN4':
        #     elif trial is 'RUN5':
        # elif skill is 'INSERTING':
        #     if trial is 'RUN0':
        #     elif trial is 'RUN1':
        # elif skill is 'PUSHING':
        #     if trial is 'RUN0':
        #     elif trial is 'RUN1':
        # elif skill is 'DRAWING':
        #     if trial is 'RUN0':
        #     elif trial is 'RUN1':


    '''
    Update the skill and trial when changed
    '''
    def change_log_settings_cb(self, msg):

        if self.logger_flag is False:

            if msg.skillName is not "":
                self.skill = msg.skillName
                rospy.loginfo("Skill name to be written is: %s" % self.skill)
        
            if msg.runName is not "":
                self.runName = msg.runName
                rospy.loginfo("Filename to be written is: %s" % self.runName)

            self.change_env(self.skill, self.runName)

            # Check to make sure the user does want to move the robot to init
            response = raw_input("Do you want to move Sawyer back to its initial position? Enter y/n \n")
            if response in ["y", "Y", 'yes', 'Yes']:
                self.limb.move_to_joint_positions(self.neutral_joint_pos)
                # self.go_to_trial_init_pos(self.trial_init_pos)  # Move Sawyer to its initial position before starting next trial
                rospy.sleep(0.5)
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


    # '''
    # Send Sawyer back into the initial position for the next trial
    # '''
    # def go_to_trial_init_pos(self, pos):
    #     # self.limb.move_to_joint_positions(self.robot_init_pos)
    #     self.limb.move_to_joint_positions(pos)



if __name__ == '__main__':
    ObjPose()
    rospy.spin()  # Prevents python from exiting until node is stopped
