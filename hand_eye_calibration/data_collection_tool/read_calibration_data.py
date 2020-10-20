#!/usr/bin/env python
import rospy
import roslib
import sys
import cv2
from curi_robotics import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from ur5_endoscope_arm.msg import *

# read joint data from robot
jsp = JointState()
def get_joint_position_from_robot(JointState1):
    global jsp
    jsp = JointState1

def usb_caliberation_node():
    rospy.init_node('usb_caliberation_node', anonymous=True)
    rospy.Subscriber("joint_states", JointState, get_joint_position_from_robot)

    frequency = 50 # 50hz
    dt = 1.0 / frequency
    loop_rate = rospy.Rate(frequency)
    
    # UR5 parameters
    UR5_joint_size = 6
    UR5_joint_type = numpy.array([0, 0, 0, 0, 0, 0, 0])
    UR5_a          = numpy.array([0, 0, -0.42500, -0.39225, 0, 0])
    UR5_alpha      = numpy.array([0, 90, 0, 0, 90, -90]) * numpy.pi / 180
    UR5_d_base     = numpy.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])
    UR5_q_base     = numpy.array([0, 0.0, 0.0, 0.0, 0.0, 0.0]) * numpy.pi / 180
    UR5            = curi_robotics(UR5_joint_size, UR5_joint_type, UR5_a, UR5_alpha, UR5_d_base, UR5_q_base, UR5_q_init)

    # set video id
    if len(sys.argv) > 1:
        print('video', sys.argv[1])
        cap = cv2.VideoCapture(int(sys.argv[1])) 
    else:
        cap = cv2.VideoCapture(1)
    
    count = 0
    last_value = 0
    vakvalueue = 0
    while not rospy.is_shutdown():
        q[0:6] = jsp.position
        bTe    = UR5.fk(q)
        print('bTe', bTe)
        
        ret, image = cap.read()
        if image is not None:
            cv2.imshow('Img', image)
        
        if ur5_panel.CMD == 'initial':
            flag = 0
        elif ur5_panel.CMD == 'control': # real robot
            if value != last_value:
                count = count + 1
                cv2.imwrite("MakerInCamera" + str(count) + ".jpg", image)

                fo = open("Image" + str(count) + ".txt", "w")
                fo.write(str(bTe[0, 0]) + " " + str(bTe[0, 1]) + " " + str(bTe[0, 2]) + " " + str(bTe[0, 3]) + "\n" + 
                         str(bTe[1, 0]) + " " + str(bTe[1, 1]) + " " + str(bTe[1, 2]) + " " + str(bTe[1, 3]) + "\n" + 
                         str(bTe[2, 0]) + " " + str(bTe[2, 1]) + " " + str(bTe[2, 2]) + " " + str(bTe[2, 3]) + "\n" + 
                         str(bTe[3, 0]) + " " + str(bTe[3, 1]) + " " + str(bTe[3, 2]) + " " + str(bTe[3, 3]) + "\n")
                fo.close()
        value = cv2.waitKey(20)
        if value & 0xFF == ord('q'):
            break
        last_value = value
        
    cap.release() 
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        usb_caliberation_node()
    except rospy.ROSInterruptException:
        pass