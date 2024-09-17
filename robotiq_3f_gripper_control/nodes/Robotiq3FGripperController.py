#!/usr/bin/env python3


# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 3F gripper gripper.

This serves as an example for publishing messages on the 'Robotiq3FGripperRobotOutput' topic using the 'Robotiq3FGripper_robot_output' msg type for sending commands to a 3F gripper gripper. In this example, only the simple control mode is implemented. For using the advanced control mode, please refer to the Robotiq support website (support.robotiq.com).
"""

from __future__ import print_function

import roslib;

roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput, Robotiq3FGripperRobotInput
from std_msgs.msg import Float32MultiArray, Float64
from Robotiq3FGripperStatusListener import printStatus, statusInterpreter
import numpy as np

try:
    input = raw_input
except NameError:
    pass

Simulation_controller_topic = ["/fanuc_gazebo/joint0_finger_A_position_controller/command",
                               "/fanuc_gazebo/joint1_finger_A_position_controller/command",
                               "/fanuc_gazebo/joint2_finger_A_position_controller/command",
                               "/fanuc_gazebo/joint3_finger_A_position_controller/command",
                               "/fanuc_gazebo/joint0_finger_B_position_controller/command",
                               "/fanuc_gazebo/joint1_finger_B_position_controller/command",
                               "/fanuc_gazebo/joint2_finger_B_position_controller/command",
                               "/fanuc_gazebo/joint3_finger_B_position_controller/command",
                               "/fanuc_gazebo/joint0_finger_C_position_controller/command",
                               "/fanuc_gazebo/joint1_finger_C_position_controller/command",
                               "/fanuc_gazebo/joint2_finger_C_position_controller/command",
                               "/fanuc_gazebo/joint3_finger_C_position_controller/command",]


class Gripper():
    def __init__(self):
        self.pub_command = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput,queue_size=1)
        self.sub_state = rospy.Subscriber("Robotiq3FGripperRobotInput", Robotiq3FGripperRobotInput, self.gripper_state_callback)
        
        self.pub_joint_sim = []
        for i in range(len(Simulation_controller_topic)):
            self.pub_joint_sim.append(rospy.Publisher(Simulation_controller_topic[i], Float64, queue_size=1))
        self.pub_joint = rospy.Publisher('Robotiq3FGripperJoint', Float32MultiArray, queue_size=12)
        
        self.sub_goal = rospy.Subscriber("Robotiq3FGripperGoal", Float32MultiArray, self.gripper_goal_callback)
        
        self.state = Robotiq3FGripperRobotInput();
        self.command = Robotiq3FGripperRobotOutput();
        self.goal = np.zeros(4) # target POA, POB, POC, POS
        self.joint = np.zeros(12) # joint angle of 12 finger joints for gazebo and avp simulation 
        

    def gripper_state_callback(self, data):
        self.state = data
        print(statusInterpreter(self.state))
        
    def gripper_goal_callback(self, data):
        for i in range(4):
            self.goal[i] = data.data[i]
        
        self.joint = self.controlGripperRegister(POA = self.goal[0],
                                                 POB = self.goal[1],
                                                 POC = self.goal[2],
                                                 POS = self.goal[3],
                                                 ICF = True)
        
    def controlGripperRegister(self, POA = 0, POB = 0, POC = 0, POS = 137, ICF = False):
        '''
            POA degree of finger A (0, 255)
            POB degree of finger B (0, 255)
            POC degree of finger C (0, 255)
            POS degree of scissor (0, 255)
            ICF whether control the degree of each finger indenpendently, 
                when ICF is False, POB and POC is not used
        '''
        COEF_SCISSORS = 26 / 220
        COEF_JOINT_1 = 62 / 140
        COEF_JOINT_2 = 90 / 100
        
        # position of scissor axis
        fa0 = 0
        fb0 = COEF_SCISSORS * min(POS, 220) - 10
        fc0 = -fb0

        # position of finger A
        fa1 = COEF_JOINT_1 * min(POA, 140)
        fa2 = COEF_JOINT_2 * min(max(POA-140, 0), 100)
        fa3 = - COEF_JOINT_1 * min(POA, 110)

        # individual control of fingers
        if ICF == False:
            fb1, fb2, fb3 = fa1, fa2, fa3
            fc1, fc2, fc3 = fa1, fa2, fa3
        else:
            fb1 = COEF_JOINT_1 * min(POB, 140)
            fb2 = COEF_JOINT_2 * min(max(POB-140, 0), 100)
            fb3 = - COEF_JOINT_1 * min(POB, 110)
            fc1 = COEF_JOINT_1 * min(POC, 140)
            fc2 = COEF_JOINT_2 * min(max(POC-140, 0), 100)
            fc3 = - COEF_JOINT_1 * min(POC, 110)
            
        commandGripper = np.array([fa0, fa1, fa2, fa3,
                                   fb0, fb1, fb2, fb3,
                                   fc0, fc1, fc2, fc3])
    
        return commandGripper
    
    def gripper_activate(self):
        # Actative the real gripper before send command
        
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSPA = 255
        self.command.rFRA = 150
        self.command.rSPB = 255
        self.command.rFRB = 150
        self.command.rSPC = 255
        self.command.rFRC = 150
        self.command.rSPS = 255
        self.command.rFRS = 150
        self.command.rICF = 1
        self.command.rICS = 1
        self.gripper_publish_real()
        
    def gripper_control(self):
        # Control the real gripper with goal position
        
        self.command.rPRA = min(max(int(self.goal[0]), 0), 255)
        self.command.rPRB = min(max(int(self.goal[1]), 0), 255)
        self.command.rPRC = min(max(int(self.goal[2]), 0), 255)
        self.command.rPRS = min(max(int(self.goal[3]), 0), 255)

        self.gripper_publish_real()

    def gripper_publish_joint(self):
        # Calculate the angle of each joint (12 in total)
        # And publish for the position control in simulation 
        
        for i in range(len(Simulation_controller_topic)):
            joint_sim_msg = Float64()
            joint_sim_msg.data = self.joint[i] / 180 * np.pi
            self.pub_joint_sim[i].publish(joint_sim_msg)
            
        joint_msg = Float32MultiArray()
        joint_msg.data = self.joint
        self.pub_joint.publish(joint_msg)
        
        # rospy.sleep(0.1)

        
    def gripper_publish_real(self):
        
        self.pub_command.publish(self.command)
        # rospy.sleep(0.1)



def main():
    """Main loop which requests new commands and publish them on the Robotiq3FGripperRobotOutput topic."""

    rospy.init_node('Robotiq3FGripperSimpleController')

    gripper = Gripper()
    print("Controller start")
    while not rospy.is_shutdown():

        if gripper.state.gIMC != 3:
            # if not activated first do the activation
            gripper.gripper_activate()
        else:
            gripper.gripper_control()
            
        gripper.gripper_publish_joint()


if __name__ == '__main__':
    main()
