import sys
import cv2
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import os
import numpy

class MoveGroupInteface(object):
    def __init__(self):
        super(MoveGroupInteface, self).__init__()
        ######################### setup ############################
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur_move_test_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()  # Not used in this tutorial
        group_name = "manipulator"  # group_name can be find in ur5_moveit_config/config/ur5.srdf
        self.move_group_commander = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        
        ################ Getting Basic Information ######################
        self.planning_frame = self.move_group_commander.get_planning_frame()
        print "============ Planning frame: %s" % self.planning_frame
        self.eef_link = self.move_group_commander.get_end_effector_link()
        print "============ End effector link: %s" % self.eef_link
        self.group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", self.robot.get_group_names()
        print "============ Printing robot state:"
        print self.robot.get_current_state()  # get
        print ""

        self.move_group_commander.set_pose_reference_frame('base_link')
        
        self.move_group_commander.set_goal_position_tolerance(0.001)
        self.move_group_commander.set_goal_orientation_tolerance(0.001)
        
        self.move_group_commander.set_max_acceleration_scaling_factor(0.5)
        self.move_group_commander.set_max_velocity_scaling_factor(0.5) 

        #self.move_group_commander.set_named_target('home')
        #self.move_group_commander.go()
        #rospy.sleep(1)

    def plan_cartesian_path(self, txyz):
        waypoints = []
        wpose = self.move_group_commander.get_current_pose().pose
        cnt = 10
        dx = (txyz[0] - wpose.position.x) / cnt
        dy = (txyz[1] - wpose.position.y) / cnt
        dz = (txyz[2] - wpose.position.z) / cnt
        for i in range(cnt):
            wpose.position.x += dx 
            wpose.position.y += dy 
            wpose.position.z += dz 
            waypoints.append(copy.deepcopy(wpose))    

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group_commander.compute_cartesian_path(waypoints,   # waypoints to follow
            0.01,      # eef_step  
            0.0)         # jump_threshold  

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        # print "=========== Planning completed, Cartesian path is saved============="
        # print('plan plan', plan)
        return plan, fraction

    def execute_plan(self, plan):
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        self.move_group_commander.execute(plan, wait=True)

    def get_pose(self):
        wpose = self.move_group_commander.get_current_pose().pose
        position = wpose.position
        orientation = wpose.orientation
        px = position.x
        py = position.y
        pz = position.z
        wx = orientation.x
        wy = orientation.y
        wz = orientation.z
        ww = orientation.w
        T = numpy.array([[2*(ww*ww + wx*wx)-1, 2*(wx*wy - ww*wz),   2*(wx*wz + ww*wy),   px],
                         [2*(wx*wy + ww*wz),   2*(ww*ww + wy*wy)-1, 2*(wy*wz - ww*wx),   py],
                         [2*(wx*wz - ww*wy),   2*(wy*wz + ww*wx)-1, 2*(ww*ww + wz*wz)-1, pz],
                         [0,                   0,                   0,                   1]])
        return T 

    def position_from_camera_to_robot(x, y, z):
        xx =  0.5659 * x + 0.8243 * y + 0.0152 * z - 0.6821
        yy = -0.3901 * x + 0.2514 * y + 0.8858 * z + 0.1501
        zz =  0.7264 * x - 0.5072 * y + 0.4638 * z + 0.6325
        return [xx, yy, zz]

tutorial = MoveGroupInteface()

frequency = 50 # 50hz
dt = 1.0 / frequency
loop_rate = rospy.Rate(frequency)

# set video id
if len(sys.argv) > 1:
    print('video', sys.argv[1])
    cap = cv2.VideoCapture(int(sys.argv[1])) 
else:
    cap = cv2.VideoCapture(1)

count = 0
last_value = 0
save_value = 0
while not rospy.is_shutdown():
    ret, image = cap.read()
    if image is not None:
        cv2.imshow('Img', image)
    else:
        randomByteArray = bytearray(os.urandom(120000))
        flatNumpyArray = numpy.array(randomByteArray)
        grayImage = flatNumpyArray.reshape(300, 400)
        cv2.imshow('Img', grayImage)

    bTe = tutorial.get_pose()
    if save_value != last_value:
        count = count + 1
        cv2.imwrite(str(count) + ".jpg", image)

        fo = open("Image" + str(count) + ".txt", "w")
        fo.write(str(bTe[0, 0]) + " " + str(bTe[0, 1]) + " " + str(bTe[0, 2]) + " " + str(bTe[0, 3]) + "\n" + 
                 str(bTe[1, 0]) + " " + str(bTe[1, 1]) + " " + str(bTe[1, 2]) + " " + str(bTe[1, 3]) + "\n" + 
                 str(bTe[2, 0]) + " " + str(bTe[2, 1]) + " " + str(bTe[2, 2]) + " " + str(bTe[2, 3]) + "\n" + 
                 str(bTe[3, 0]) + " " + str(bTe[3, 1]) + " " + str(bTe[3, 2]) + " " + str(bTe[3, 3]) + "\n")
        fo.close()
        print(save_value, last_value)

    value = cv2.waitKey(50)
    if value & 0xFF == ord('q'):
        break
    last_value = save_value
    if value > 0:
        print(value)
        save_value = value
        
cap.release() 
cv2.destroyAllWindows()


