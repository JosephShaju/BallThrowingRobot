#! /usr/bin/env python

from geometry_msgs import msg as geoMsg
from nav_msgs import msg as navMsg
import rospy
import numpy
from scipy.linalg import pinv
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
rospy.init_node('move_robot_using_trajectory_msg')
import sys
from geometry_msgs.msg import Pose 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import pi
from math import cos
from math import sin
from sympy import *
from robot_functions import *

JOINT_NAMES = ["j2n6s300_joint_1","j2n6s300_joint_2","j2n6s300_joint_3","j2n6s300_joint_4","j2n6s300_joint_5","j2n6s300_joint_6"]

currentJointState = Pose()
pose = geoMsg.Pose()
def jointStatesCallback(msg):
    global currentJointState
    currentJointState = msg

topic_name = '/j2n6s300/joint_states'

sub = rospy.Subscriber(topic_name, JointState, jointStatesCallback)
rospy.sleep(3)


def odom():
    return navMsg.Odometry()
go = odom()


print(currentJointState.position)
print("-------------------------\n")
    
if __name__ == '__main__':
  try:
    ti = time.time()
    rospy.init_node('move_robot_using_trajectory_msg')

    # Allow gazebo to launch
    rospy.sleep(1)

    ######################### EDIT THIS SECTION ###############################
    # Parameters
    numPts = 4		# Discretized path (Choose number of points)
    step   = 1.0	# Step size 
    error  = 0.001	# Error between state and desired state (theshold value, in meters)
    firstMove = True
    # Initial state
    q10 = currentJointState.position[0]
    q20 = currentJointState.position[1]
    q30 = currentJointState.position[2]
    q40 = currentJointState.position[3]
    q50 = currentJointState.position[4]
    q60 = currentJointState.position[5]
    #1.6962834294765.   1.66283127117. 1.67592124617
    #graspBall1 = [-1.7239178628693832, 1.6962834294765, 3.3961112257191512, -3.38462117823865, 1.4459102321300366, 0.20018753676786538]
    graspBallLeft = [-3.29471786287, 1.67592124617, 3.39262056572, -3.38462117823865, 1.4459102321300366, 0.20018753676786538]
    graspBallLeft2 = [-3.29471786287, 1.67592124617, 3.39262056572, -3.38462117823865, 1.4459102321300366, 0.20018753676786538]
    graspBallRight = [-0.15311786286, 1.676648467, 3.3961112257191512, -3.38462117823865, 1.4459102321300366, 0.20018753676786538]
    

    #8.309801772021785e-05, 2.9006601314232663, 1.2986910653187502, -2.07982669269741, 1.401891180285613, -1.8971810852086435e-05, 1.000071137932979, 1.0011800646481124, 0.9996488391669853, 0.00953578528302046, 0.0018044499440588169, 0.001533493570360811
    #graspBallRight2 = [-0.15311786286, 1.676648467, 3.3961112257191512, -3.38462117823865, 1.4459102321300366, 0.20018753676786538]
    graspBallNormal = [-1.7239178628693832, 1.66283127117, 3.3961112257191512, -3.38462117823865, 1.4459102321300366, 0.20018753676786538]
    throwPosn = [-1.7239178628693832, 2.5543, 4.3961112257191512, -3.38462117823865, 1.4459102321300366, 0.20018753676786538]

    keepPosnRight= [-0.15311786286, 2.5543, 4.3961112257191512, -3.38462117823865, 1.4459102321300366, 0.20018753676786538]
    keepPosnLeft= [-0.15311786286, 2.5543, 4.3961112257191512, -3.38462117823865, 1.4459102321300366, 0.20018753676786538]

    qleft1 = graspBallLeft[0]
    qleft2 = graspBallLeft[1]
    qleft3 = graspBallLeft[2]
    qleft4 = graspBallLeft[3]
    qleft5 = graspBallLeft[4]
    qleft6 = graspBallLeft[5]

    # -0.25 in x and +0.75 in z 0.375 in y top ledge
    # +0.75 in x, +0.5 in z and 0.375 in y bottom ledge

    qright1 = graspBallRight[0]
    qright2 = graspBallRight[1]
    qright3 = graspBallRight[2]
    qright4 = graspBallRight[3]
    qright5 = graspBallRight[4]
    qright6 = graspBallRight[5]

    q13 = graspBallNormal[0]
    q23 = graspBallNormal[1]
    q33 = graspBallNormal[2]
    q43 = graspBallNormal[3]
    q53 = graspBallNormal[4]
    q63 = graspBallNormal[5]

    t1 = throwPosn[0]
    t2 = throwPosn[1]
    t3 = throwPosn[2]
    t4 = throwPosn[3]
    t5 = throwPosn[4]
    t6 = throwPosn[5]

    # gripperGrasp = [0.7, 0.6, 0.5]


    def origPosition():
        moveJoint([q10,q20,q30,q40,q50,q60])
        moveFingers([1.3,1.3,1.3])
    
    def throwPosition(gripPosn):
        moveFingers(gripPosn)
        moveJoint([t1,t2,t3,t4,t5,t6])
    
    def initPositionWithBall(gripPosn):
        moveFingers(gripPosn)       
        moveJoint([q10,q20,q30,q40,q50,q60])
    
    def aboveBallPosition1():
        moveJoint([q13,q23,q33,q43,q53,q63])
    
    def aboveLeftBallPosition():
        moveJoint([qleft1,qleft2,qleft3,qleft4,qleft5,qleft6])
    
    def aboveRightBallPosition():
        moveJoint([qright1,qright2,qright3,qright4,qright5,qright6])

    def gripBall1():
        grip = [1.055, 1.065, 1.065]
        moveFingers(grip)
        return grip
    
    def gripBall2():
        grip = [1.05, 1.06, 1.06]
        moveFingers(grip)
        return grip


    # Move robot to initial state
    # moveJoint([q10,q20,q30,q40,q50,q60])
    moveFingers([0.3,0.3,0.3])
    print("Grip or Move?")
    i = raw_input()
    while (i == "m"):
        print("Which position?")
        b = raw_input()
        if b == "l":
            aboveLeftBallPosition()
        if b == "r":
            aboveRightBallPosition()
        if b == "lt":
            cont = "y"
            angadd1=0
            angadd2=0
            angadd3=0
            angadd4=0
            angadd5=0
            angadd1 += -1*float(raw_input("x degrees Upward J2: "))
            ltleft1 = -3.29471786287
            ltleft2 = 1.67592124617+(90*pi/180)+(angadd1*pi/180)
            ltleft3 = 3.39262056572+(angadd2*pi/180)
            ltleft4 = -3.38462117823865-(angadd3*pi/180)
            ltleft5 = 1.4459102321300366+(angadd4*pi/180)
            ltleft6 = 0.20018753676786538+(angadd5*pi/180)
            cont = raw_input("Next(any/n): ")
            moveJoint([ltleft1,ltleft2,ltleft3,ltleft4,ltleft5,ltleft6])
            ltleft1 = -3.29471786287+0.717541341
            moveJoint([ltleft1,ltleft2,ltleft3,ltleft4,ltleft5,ltleft6])

            while cont != "n":
                angadd1 += -1*float(raw_input("x degrees Upward J2: "))
                ltleft2 = 1.67592124617+(90*pi/180)+(angadd1*pi/180)
                moveJoint([ltleft1,ltleft2,ltleft3,ltleft4,ltleft5,ltleft6])
                angadd2 += float(raw_input("x degrees Upward J3: "))
                ltleft3 = 3.39262056572 +(angadd2*pi/180)
                #ltleft1 = -3.29471786287+(angadd*pi/180)
                moveJoint([ltleft1,ltleft2,ltleft3,ltleft4,ltleft5,ltleft6])
                angadd3 += -1*float(raw_input("Rot x degrees J4: "))
                ltleft4 = -3.38462117823865+(angadd3*pi/180)
                moveJoint([ltleft1,ltleft2,ltleft3,ltleft4,ltleft5,ltleft6])
                angadd4 += -1*float(raw_input("Rot x degrees J5: "))
                ltleft5 = 1.4459102321300366+(angadd4*pi/180)
                moveJoint([ltleft1,ltleft2,ltleft3,ltleft4,ltleft5,ltleft6])
                angadd5 += -1*float(raw_input("Rot x degrees J6: "))
                ltleft6 = 0.20018753676786538+(angadd5*pi/180)
                moveJoint([ltleft1,ltleft2,ltleft3,ltleft4,ltleft5,ltleft6])
                ltleft1 += (float(raw_input("Rotate base by (degrees): "))*pi/180)
                moveJoint([ltleft1,ltleft2,ltleft3,ltleft4,ltleft5,ltleft6])

                ########################################################
                ###                 ANGLES                           ###
                ########################################################
                #(-35.0, -38.0, -101.0, 113.0, 53.0)

                print("Angles 2-6")
                print(angadd1,angadd2,angadd3,angadd4,angadd5)               
                print("Base angle: ")
                print(ltleft6)
                cont = raw_input("continue? (any/n): ")            


        print("Move?")
        i = raw_input()

    print("Grip?")
    j = raw_input()
    while (j == "g"):
        print("Grip ball?")
        b = raw_input()
        if b == "1":
            gripPosn = gripBall1()
        if b == "2":
            gripPosn = gripBall2()
        print("Grip?")
        j = raw_input()

    print("Move back to home?")
    ans = raw_input()
    if ans == "y":
        origPosition()
    else:
        throwPosition(gripPosn)
    '''
    # Force control ??
    # Final state
    xf = 0.0
    yf = 0.75
    zf = 0.5
    ###########################################################################
    # Boundary conditions (DON'T WORRY ABOUT THESE, and DON'T UNCOMMENT)
    init_q  = Matrix([[-q10], [q20-pi/2], [q30+pi/2], [q40], [q50-pi], [q60+pi/2]])
    #init_q1  = Matrix([[-q11], [q21-pi/2], [q31+pi/2], [q41], [q51-pi], [q61+pi/2]])
    
    # Initial position
    _, init_X = systemKinematics(init_q)
    #$_, init_X1 = systemKinematics(init_q)
    final_X = Matrix([[xf],[yf],[zf]])
    
    print(init_X[0], final_X[0])
    print(type(init_X[0]), type(final_X[0]))
    ######################### EDIT THIS SECTION ###############################
    # Discretize path (change this to change path shape, default is line)
    xPath = numpy.linspace(float(init_X[0]), float(final_X[0]), num=numPts)
    yPath = numpy.linspace(float(init_X[1]), float(final_X[1]), num=numPts)
    zPath = numpy.linspace(float(init_X[2]), float(final_X[2]), num=numPts)
    ###########################################################################
    # Initialize variables
    delta_X = Matrix([[1000],[1000],[1000]])
    
    X = []
    Y = []
    Z = []
    # Matrix calculatations
    dynMatrix,_ = systemKinematics(init_q)
    row0 = dynMatrix.row(0)
    row1 = dynMatrix.row(1)
    row2 = dynMatrix.row(2)
    DynMatrix = Matrix([[row0],[row1],[row2]])
    J = robotJacobian(DynMatrix)
    # Connect all points in mesh path (START LOOP AT 1, NOT 0)
    for i in range(1,numPts):
      path_X = Matrix([[xPath[i]],[yPath[i]],[zPath[i]]])
      print(i)
      _, init_X = systemKinematics(init_q)
      delta_X = path_X - Matrix([[init_X[0]],[init_X[1]],[init_X[2]]])
      moveFingers([1.05, 1.08, 1.08])
      while (Abs(delta_X.norm()) > error):
        count = 0
        _, init_X = systemKinematics(init_q)
        delta_X = path_X - Matrix([[init_X[0]],[init_X[1]],[init_X[2]]])
        print(Abs(delta_X.norm()))
        count +=1
        # Linearize dynamics matrix
        useful_J = evalJacobian(J,init_q)
        # Calculate joint state change and new state
        delta_q = inverseKinematics(useful_J,step,delta_X)
        init_q = init_q + step*delta_q
        # Convert back from DH angles to JACO angles
        moveFingers([1.05, 1.08, 1.08])
        moveJoint([-init_q[0], init_q[1]+pi/2, init_q[2]-pi/2, init_q[3], init_q[4]+pi, init_q[5]-pi/2])
        
        # Trajectory plotting
        X.append(init_X[0])
        Y.append(init_X[1])
        Z.append(init_X[2])
 
    #moveFingers([0.9,0.9,0.9])
    elapsed = time.time() - ti
    print(elapsed)
    # fig = plt.figure()
    # ax = fig.gca(projection='3d')    
    # ax.plot(X, Y, Z, label='traveled')
    # ax.plot(xPath, yPath, zPath, label='path')
    # ax.legend()
    # plt.show()
    '''
  except rospy.ROSInterruptException:
    print "program interrupted before completion"
