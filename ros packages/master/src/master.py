#!/usr/bin/env python3
from goemetry_msgs.msg import Pose
from std_msgs.msg import Bool
import roslib
roslib.load_manifest('master') 
import actionlib
import time
import rospy
import igvc_action_server.msg
import numpy as np
from std_msgs.msg import Float32MultiArray, Bool
from cv_client1 import upward_movement_client
from cv_client2 import port_detection_client
from cv_client3 import port_detection_client_transit

state_list = ['s0', 's1', 's2', 's3', 's4', 's5', 's6', 's7', 's8']

def master():
    def __init__(self):
        self.charging_trigger = False
        self.current_state = None
        self.current_pose = False
        self.car_detected = False
        self.goal_pose = None #goal 4x4 matrix
        self.goal_pose_s2 = None #goal 4x4 matrix
        self.goal_pose_cv_1 = None #goal 4x4 matrix
        self.goal_pose_cv_2 = None #goal 4x4 matrix
        self.latched = False
        #self.initial_pose = None


        self.home_pose = 
        self.after_turn_pose = 
        self.scan_pose = 


        self.port_detected = False
        pose_sub = rospy.Subsriber('/gazebo/robot_position', Float32MultiArray, self.pose_CB)
        cv_goal_pos_sub = rospy.Subscriber('/cv/pose', Float32MultiArray, self.cv_goal_cb)
        cv_port_det = rospy.Subscriber('/cv/port_detect', Bool, self.cv_port_cb)
        pose_pub_final = rospy.Publisher('/planning/goal_pose', Float32MultiArray, queue_size = 10)
        pose_pub_current = rospy.Publisher('/planning/current_pose', Float32MultiArray, queue_size = 10)

    def mat_comp(mat1, mat2):
        if mat1 == mat2 :
            return(True) # compare with threshold need to tune

    def cv_goal_cb(self, msg):
        self.goal_pose_cv = msg

    def six_to_4x4(six_x_1):
        #forward kinematics
        pass

    def flat_to_4x4(mat):
        m = np.reshape(mat, (4,4))
        return m

    def flat_from_4x4(mat):
        m = np.reshape(mat, 16)
        return m

    def pose_CB(self, msg):
        pose = msg
        #forward kinematics
        self.current_pose = six_to_4x4(msg)

    def approximate_goal(self, pose):
    	# go slighly backwards in the frame orientation for approximate goal

    def run(self):
        i = 0
        while (i<len(self.state_list)):

            self.current_task = self.state_list[i]

            if (self.current_task == 's0'):
                if self.charging_trigger == True:
                    i+=1

            elif (self.current_task == 's1'):
                #need to publish pose to controls
                self.goal_pose = self.after_turn_pose
                self.pose_pub_final.publish(self.goal_pose)
                self.pose_pub_current.publish(self.current_pose)
                #NEED TO DEFINE GOAL POSE
                if (mat_comp(self.pose_CB, self.goal_pose)):
                    i+=1
###################################################################################################################
            elif (self.current_task == 's2'):
                #call Computer vision server for car detection and use client function - will implement
                #feedback - car detected
                #call car detection client function
                pose_pub.publish(self.flat_from_4x4(self.goal_pose_s2))
                upward_movement_client(self.goal_pose_cv_2)
                i+=1

            elif (self.current_task == 's3'):
                #call computer vision server for port detection - implement client too
                #feedback - port detected
                #call charging port detection client
                port_detection_client(True)
                i+=1 #COMBINE S2 S3 
##################################################################################################################### 
            elif (self.current_task == 's4'):
                #implement function to compare current pose with goal pose from CV and make it 90 degs
                #need to publish pose to controls
                port_detection_client(True) #make location as result
                #final goal -> approx goal
                self.goal_pose = self.approximate_goal(self.goal_pose_cv_1) 

                pose_pub.publish(self.goal_pose)
                if (self.mat_comp(self.current_pose,self.goal_pose)):
                    i+=1

            elif (self.current_task == 's5'):
                #result CV - raises flag - port not detected
                #might need to remove collision check
                s = True
                while(s):
                    m = port_detection_client_transit(True)
                    r = m.new_goal
                    pose_pub.publish(r)
                    s = m.can_see_charging_port
                #need server-client here too
                if (s == False):
                    self.goal_pose_cv_2 = r
                    i+=1

            elif (self.current_task == 's6'):

                # if (self.mat_comp(self.goal_pose_cv_2,self.current_pose)):
                time.sleep(30)
                i+=1

            elif (self.current_task == 's7'):
                #calculate position to go back to based on current position from gazebo
                #fb - current position from gazebo
                for i in [1]:
                	self.goal pose = approximate_goal(self.current_pose)
                pose_pub_final.publish(self.goal_pose)
                pose_pub_current.publish(self.current_pose)

                if (self.mat_comp(self.current_pose,self.goal_pose)):
                	i+=1

            elif (self.current_task == 's8'):
                #take current position of car and compare with initial position
                #when equal, go to reset state

                self.goal_pose = self.home_pose
                pose_pub_final.publish(self.goal_pose)
                pose_pub_current.publish(self.current_pose)

                if (self.mat_comp(self.current_pose,self.home_pose)):
                    i = 0
