#!/usr/bin/env python3
import sys,os
sys.path.append(os.path.join(os.path.dirname(__file__),'../../../devel/lib/python3/dist-packages/'))

import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
import copy
from robotiq_3f_gripper_control.srv import gripper_action,gripper_actionResponse
def genCommand(char, command):
    """Update the command according to the character entered by the user."""

    if char == 'a':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if char == 'r':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 0

    if char == 'c':
        command.rPRA = 255

    if char == 'o':
        command.rPRA = 0

    if char == 'b':
        command.rMOD = 0

    if char == 'p':
        command.rMOD = 1

    if char == 'w':
        command.rMOD = 2

    if char == 's':
        command.rMOD = 3

    # If the command entered is a int, assign this value to rPRA
    try:
        command.rPRA = int(char)
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass

    if char == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255

    if char == 'l':
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0

    if char == 'i':
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255

    if char == 'd':
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0

    return command


def askForCommand(command):
    """Ask the user for a command to send to the gripper."""

    currentCommand = 'Simple 3F gripper Controller\n-----\nCurrent command:'
    currentCommand += ' rACT = ' + str(command.rACT)
    currentCommand += ', rMOD = ' + str(command.rMOD)
    currentCommand += ', rGTO = ' + str(command.rGTO)
    currentCommand += ', rATR = ' + str(command.rATR)
    currentCommand += ', rPRA = ' + str(command.rPRA)
    currentCommand += ', rSPA = ' + str(command.rSPA)
    currentCommand += ', rFRA = ' + str(command.rFRA)

    print(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 'b: Basic mode\n'
    strAskForCommand += 'p: Pinch mode\n'
    strAskForCommand += 'w: Wide mode\n'
    strAskForCommand += 's: Scissor mode\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)

command = Robotiq3FGripperRobotOutput()
def handle_gripper_action(req:gripper_action):
    global command
    print('gripper action',chr(req.gripper_state.data))
    command = genCommand(chr(req.gripper_state.data), command)
    gripper_pub.publish(command)
    return gripper_actionResponse(True)
    
if __name__=='__main__':
    # def publisher():
    print('starting Robotiq3FGripperController')
    rospy.init_node('Robotiq3FGripperController')
    print('Robotiq3FGripperController started')
    rospy.loginfo("""Main loop which requests new commands and publish them on the Robotiq3FGripperRobotOutput topic.""")

    # print('succ init')
    # fanuc = Robot()
    # fanuc.set_robot_travel_time(5)
    gripper_pub = rospy.Publisher('Robotiq3FGripperRobotOutput', 
    Robotiq3FGripperRobotOutput,queue_size=1)
    print('handle_gripper_action')
    s=rospy.Service('/gripper_action',gripper_action,handler=handle_gripper_action)
    rospy.spin()
