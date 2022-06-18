#!/usr/bin/python

import numpy as np
from ik_UR5 import IK_optimal, ik_UR5
import matplotlib.pyplot as plt
from time import sleep
import matplotlib.pyplot as plt
import csv


PI = np.pi

# def fifth_poly_plan(q0:float, qf:float, rate:int, time:int, v0:float = 0, vf:float = 0, ac0:float = 0, acf:float = 0)->list:
def fifth_poly_plan(q0, qf, rate, time, v0 = 0, vf = 0, ac0 = 0, acf = 0):
    '''
    @param rate: ros loop rate (Hz)
    time: total time of motion (s). And steps = time*rate
    '''
    a0 = q0
    a1 = v0
    a2 = ac0/2
    a3 = (20*qf - 20*q0 -(8*vf + 12*v0)*time - (3*ac0 - acf)*time*time)/(2*time**3)
    a4 = (30*q0 - 30*qf +(14*vf + 16*v0)*time + (3*ac0 - 2*acf)*time*time)/(2*time**4)
    a5 = (12*qf - 12*q0 -(6*vf + 6*v0)*time + (ac0 - acf)*time*time)/(2*time**5)
    joint_traj = []
    for t in range(rate * time):
        t = t/float(rate)   # step
        q = a0 + a1*t + a2* t*t + a3* t**3 + a4* t**4 + a5* t**5
        joint_traj.append(q)
    return joint_traj

def six_axes_fifth_poly_plan(initial, final, rate, time):
    '''
    initial: 1x6 joint angle vector
    '''
    if len(initial) != len(final):
        raise("Inconsistent size of joint vectors.")
    joints_traj = []
    for i in range(len(initial)):
        q = fifth_poly_plan(initial[i], final[i], rate, time)
        joints_traj.append(q)
    return np.array(joints_traj).transpose()


def single_axis_test():
    joint_traj = fifth_poly_plan(0, 2*PI, 30, 5)
    time = np.linspace(0, 5, 30*5)
    plt.plot(time, joint_traj)
    plt.show()

def six_axes_test():
    final = [PI/2, -PI/2, PI/4, 3*PI/4, PI, -PI]
    joints_traj = six_axes_fifth_poly_plan([0,0,0,0,0,0], final, 30, 5)
    time = np.linspace(0, 5, 150)
    q1,q2,q3,q4,q5,q6 = [],[],[],[],[],[]
    for i in range(len(joints_traj)):
        q1.append(joints_traj[i][0])
        q2.append(joints_traj[i][1])
        q3.append(joints_traj[i][2])
        q4.append(joints_traj[i][3])
        q5.append(joints_traj[i][4])
        q6.append(joints_traj[i][5])
    plt.plot(time, q1)
    plt.plot(time, q2)
    plt.plot(time, q3)
    plt.plot(time, q4)
    plt.plot(time, q5)
    plt.plot(time, q6)
    plt.show()

def main_ros():
    import rospy
    from sensor_msgs.msg import JointState
    rospy.init_node("joint_angle_publisher")
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1000)
    RATE = 30
    rate = rospy.Rate(RATE)
    jointState = JointState()
    jointState.name = ['left_shoulder_pan_joint', 'left_shoulder_lift_joint', 'left_elbow_joint', 'left_wrist_1_joint', 'left_wrist_2_joint', 'left_wrist_3_joint', 'right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint', 'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']

# IN world frame {W}
    
    TILT = -3*PI/4  # base_link
    # transform from {B1} to {W}
    T_W_B1 = np.array([
        [np.cos(TILT), -np.sin(TILT), 0, 0],
        [np.sin(TILT), np.cos(TILT), 0, -0.66],
        [0,0,1,0],
        [0,0,0,1]])
    T_B1_W = np.linalg.inv(T_W_B1)
    # transform from {B2} to {W}
    T_W_B2 = np.array([
        [np.cos(TILT), -np.sin(TILT), 0, 0],
        [np.sin(TILT), np.cos(TILT), 0, 0.66],
        [0,0,1,0],
        [0,0,0,1]
    ])
    T_B2_W = np.linalg.inv(T_W_B2)
    np.set_printoptions(precision=3, suppress=True)
    # print(T_B2_W)
    
   
    T1_W = np.array([0,-0.2, 0.5, 1])
    T2_W = np.array([0,0.2, 0.5, 1])
    T1_B = np.matmul(T_B1_W, T1_W)
    T2_B = np.matmul(T_B2_W, T2_W)
    # print("T1_B:",T1_B)
    # print("T2_B:",T2_B)

    pose1 = [T1_B[0], T1_B[1], T1_B[2], -PI/2, 0, 3*PI/4] 
    pose2 = [T2_B[0], T2_B[1], T2_B[2], -PI/2, 0, -PI/4]
    initialPosition = [0,-PI/2,0,0,0,0]
    joint_pos1 = IK_optimal(pose1, initialPosition)
    joint_pos2 = IK_optimal(pose2, initialPosition)
    # different zero configuration with real UR
    # joint_pos1[0] += PI
    # joint_pos1[1] += PI/2
    # joint_pos2[0] += PI
    # joint_pos2[1] += PI/2
    # print("joint position 1:", joint_pos1)
    # print("joint position 2:", joint_pos2)

    N = 1000
    t = np.linspace(0-PI/2, 2*PI-PI/2, N)
    X = 0.15 * np.cos(t)
    Z = 0.15 * np.sin(t)+0.5
    # joint_traj_data = open('/home/yinghao/catkin_ws/src/dualarm/data/joints_traj.csv','w',encoding='utf-8')
    # traj_recorder = csv.writer(joint_traj_data)
    # traj_recorder.writerow(['R1J1','R1J2','R1J3','R1J4','R1J5','R1J6','R2J1','R2J2','R2J3','R2J4','R2J5','R2J6'])

    last_pos1, last_pos2 = np.zeros(6), np.zeros(6)
    for i in range(N):
        # print(X[i], Z[i])
        T1_W = np.array([X[i],-0.2,Z[i], 1])
        T2_W = np.array([X[i],0.2, Z[i], 1])
        T1_B = np.matmul(T_B1_W, T1_W)
        T2_B = np.matmul(T_B2_W, T2_W)
        # print(T1_B[:3],T2_B[:3])
        pose1 = [T1_B[0], T1_B[1], T1_B[2], -PI/2, 0, 3*PI/4] 
        pose2 = [T2_B[0], T2_B[1], T2_B[2], -PI/2, 0, -PI/4]
        # joint_pos1 = IK_optimal(pose1, joint_pos1)
        joint_pos1 = ik_UR5(pose1)[0]
        if (joint_pos1[5] - last_pos1[5]) > 0.9*2*PI:
            joint_pos1[5] -= 2*PI
        if (joint_pos1[5] - last_pos1[5]) < -0.9*2*PI:
            joint_pos1[5] += 2*PI
        last_pos1 = joint_pos1
        # joint_pos2 = IK_optimal(pose2, joint_pos2)
        joint_pos2 = ik_UR5(pose2)[0]
        if (joint_pos2[5] - last_pos2[5]) > 0.9*2*PI:
            joint_pos2[5] -= 2*PI
        if (joint_pos2[5] - last_pos2[5]) < -0.9*2*PI:
            joint_pos2[5] += 2*PI
        last_pos2 = joint_pos2
        if i == 0:
            print("Moving from home configuration to the start of the circle")
            joints_traj1 = six_axes_fifth_poly_plan(initialPosition, joint_pos1, RATE, 10)
            joints_traj2 = six_axes_fifth_poly_plan(initialPosition, joint_pos2, RATE, 10)
            for j in range(len(joints_traj1)):
                joint_pos1 = joints_traj1[j]
                joint_pos2 = joints_traj2[j]

                jointState.header.stamp = rospy.Time.now()
                jointState.position = [joint_pos1[0], joint_pos1[1], joint_pos1[2], joint_pos1[3], joint_pos1[4], joint_pos1[5], joint_pos2[0], joint_pos2[1], joint_pos2[2], joint_pos2[3], joint_pos2[4], joint_pos2[5]]
                pub.publish(jointState)
                rate.sleep()  
                # traj_recorder.writerow(jointState.position)
        else:
            jointState.header.stamp = rospy.Time.now()
            jointState.position = [joint_pos1[0], joint_pos1[1], joint_pos1[2], joint_pos1[3], joint_pos1[4], joint_pos1[5], joint_pos2[0], joint_pos2[1], joint_pos2[2], joint_pos2[3], joint_pos2[4], joint_pos2[5]]
            pub.publish(jointState)
            rate.sleep()
    #         traj_recorder.writerow(jointState.position)
    # traj_recorder.close()

def RotZ(theta):
    cos = np.cos
    sin = np.sin
    return np.array([[cos(theta), -sin(theta), 0],
                     [sin(theta), cos(theta), 0],
                     [0,          0,          1]])
def RotX(theta):
    sin = np.sin
    cos = np.cos
    return np.array([[1,  0,          0],
                     [0,cos(theta),-sin(theta)],
                     [0,sin(theta),cos(theta)]])

def main():
    from connect_ur import UR_state, ur_connection_thread
    from forceControl import ForceSensor, ImpedanceController, sensor_connection_thread
    # Initialization of robots
    ur1 = UR_state()
    ur2 = UR_state()
    ur1.setIP("192.168.0.1")
    ur2.setIP("192.168.0.2")
    ur1.connect()
    ur2.connect()
    ur1.getState()
    ur2.getState()

    ur1_read = ur_connection_thread("ur1", ur1)
    ur1_read.setDaemon(True)
    ur1_read.start()

    ur2_read = ur_connection_thread("ur2", ur2)
    ur2_read.setDaemon(True)
    ur2_read.start()

    jointAngle1 = ur1.getJointPosition()
    jointAngle2 = ur2.getJointPosition()
    ur1.set_initial_configuration(jointAngle1)
    ur2.set_initial_configuration(jointAngle2)
    print("Initial position angle of robot1: ", jointAngle1)
    print("Initial position angle of robot2: ", jointAngle2)

    

    # MOTION PLANNING
    TILT = -3*PI/4  # base_link
    # transform from {B1} to {W}
    T_W_B1 = np.array([
        [np.cos(TILT), -np.sin(TILT), 0, 0],
        [np.sin(TILT), np.cos(TILT), 0, -0.66],
        [0,0,1,0],
        [0,0,0,1]])
    T_B1_W = np.linalg.inv(T_W_B1)
    # transform from {B2} to {W}
    T_W_B2 = np.array([
        [np.cos(TILT), -np.sin(TILT), 0, 0],
        [np.sin(TILT), np.cos(TILT), 0, 0.66],
        [0,0,1,0],
        [0,0,0,1]
    ])
    T_B2_W = np.linalg.inv(T_W_B2)
    np.set_printoptions(precision=3, suppress=True)
    # print(T_B2_W)
    
    # The equation of a circle
    N = 2000
    t = np.linspace(0-PI/2, 2*PI-PI/2, N)
    X = 0.15 * np.cos(t)
    Z = 0.15 * np.sin(t)+0.5
    

    # moving to the sides of the box
    T1_W = np.array([X[0],-0.21,Z[0], 1])
    T2_W = np.array([X[0],0.21, Z[0], 1])
    T1_B = T_B1_W @ T1_W
    T2_B = T_B2_W @ T2_W
    pose1 = [T1_B[0], T1_B[1], T1_B[2], -PI/2, 0, 3*PI/4] 
    pose2 = [T2_B[0], T2_B[1], T2_B[2], -PI/2, 0, -PI/4]
    ur1.moveCartesian(pose1)
    ur2.moveCartesian(pose2)
    print("Robot 1 and 2 moving to initial position...")
    sleep(10)

    # Initialization of force sensors
    forceSensor1 = ForceSensor('192.168.0.100')
    forceSensor1.tare()
    forceSensor1.start()
    sensor1_connect = sensor_connection_thread("sensor1", forceSensor1)
    sensor1_connect.setDaemon(True)
    sensor1_connect.start()
    # forceSensor1.tare()

    forceSensor2 = ForceSensor('192.168.0.200')
    forceSensor2.tare()
    forceSensor2.start()
    sensor2_connect = sensor_connection_thread("sensor2", forceSensor2)
    sensor2_connect.setDaemon(True)
    sensor2_connect.start()
    # forceSensor2.tare()

    # gripping
    T1_W = np.array([X[0],-0.196,Z[0], 1])
    T2_W = np.array([X[0],0.196, Z[0], 1])
    T1_B = T_B1_W @ T1_W
    T2_B = T_B2_W @ T2_W
    pose1 = [T1_B[0], T1_B[1], T1_B[2], -PI/2, 0, 3*PI/4] 
    pose2 = [T2_B[0], T2_B[1], T2_B[2], -PI/2, 0, -PI/4]
    ur1.moveCartesian(pose1)
    ur2.moveCartesian(pose2)
    print("Robot 1 and 2 gripping the box...")
    sleep(5)

    R_B1T1 = RotZ(3/4*PI) @ RotX(-PI/2)   #zyx (3/4*PI, 0, -PI/2)
    R_B2T2 = RotZ(-PI/4) @ RotX(-PI/2)   #zyx (-PI/4, 0, -PI/2)
    # print("R_B1T1\n",R_B1T1)
    # print("R_B2T2\n",R_B2T2)
    forceControl1 = False
    forceControl2 = True
    ic1 = ImpedanceController(1,200,0)
    ic2 = ImpedanceController(1,200,0)
    zt1, d_zt1, zt2, d_zt2 = 0, 0, 0, 0
    Fz2 = []

    ## record contact force
    dualrobot_f = open('/home/multiarm/catkin_ws/src/dualarm/data/robot1_fz.csv', 'w', encoding='utf-8')
    dualrobot_force_recoder = csv.writer(dualrobot_f)
    dualrobot_force_recoder.writerow(['t','fz1', 'fz2'])
    # robot2_f = open('/home/multiarm/catkin_ws/src/dualarm/data/robot2_fz.csv', 'w', encoding='utf-8')
    # robot2_force_recoder = csv.writer(robot2_f)
    # robot2_force_recoder.writerow(['t','fz'])

    # if forceControl2:
    for i in range(1,100):
        fz1 = -forceSensor1.F[2]
        fz2 = -forceSensor2.F[2]
        # Fz2.append(fz)
        dualrobot_force_recoder.writerow([0.04*i, fz1, fz2])
        print(fz1, fz2)
        if (fz1 > 30) or (fz2 > 30):
            print(f"Excessive contact force:{fz1}, {fz2}!")
            break
        # loop until Fz > 5 N
        zt1, d_zt1 = ic1.ordinaryCompute(zt1,d_zt1, fz1, 5)
        dx, dy, dz = R_B1T1 @ np.array([0,0,zt1])
        pose1 = [T1_B[0]+dx, T1_B[1]+dy, T1_B[2]+dz, pose1[3], pose1[4], pose1[5]]

        zt2, d_zt2 = ic2.ordinaryCompute(zt2, d_zt2, fz2, 5)
        dx, dy, dz = R_B2T2 @ np.array([0,0,zt2])
        pose2 = [T2_B[0]+dx, T2_B[1]+dy, T2_B[2]+dz, pose2[3], pose2[4], pose2[5]]
        ur1.servoCartesian(pose1)
        ur2.servoCartesian(pose2)
        sleep(0.04)

    ## servoing
    robot1_joint_traj= open('/home/multiarm/catkin_ws/src/dualarm/data/joint_position_1.csv', 'w', encoding='utf-8')
    robot1_joint_recorder = csv.writer(robot1_joint_traj)
    robot1_joint_recorder.writerow(['q1','q2','q3','q4','q5','q6'])
    robot2_joint_traj = open('/home/multiarm/catkin_ws/src/dualarm/data/joint_position_2.csv', 'w', encoding='utf-8')
    robot2_joint_recorder = csv.writer(robot2_joint_traj)
    robot2_joint_recorder.writerow(['q1','q2','q3','q4','q5','q6'])

    for i in range(1,N):
        # print(X[i], Z[i])
        T1_W = np.array([X[i],-0.196,Z[i], 1])
        T2_W = np.array([X[i],0.196, Z[i], 1])
        T1_B = T_B1_W @ T1_W
        T2_B = T_B2_W @ T2_W
        pose1 = [T1_B[0], T1_B[1], T1_B[2], -PI/2, 0, 3*PI/4] 
        pose2 = [T2_B[0], T2_B[1], T2_B[2], -PI/2, 0, -PI/4]
        
        if forceControl2:
            fz1 = -forceSensor1.F[2]
            fz2 = -forceSensor2.F[2]
            # Fz2.append(fz)
            dualrobot_force_recoder.writerow([3.96+0.04*i, fz1, fz2])
            # robot2_force_recoder.writerow([3.96+0.04*i, fz])
            print(fz1, fz2)
            if (fz1 > 30) or (fz2 > 30):    
                print(f"Excessive contact force:{fz1}, {fz2}!")
                break
            # loop until Fz > 5 N
            zt1, d_zt1 = ic1.ordinaryCompute(zt1,d_zt1, fz1, 5)
            dx, dy, dz = R_B1T1 @ np.array([0,0,zt1])
            pose1 = [T1_B[0]+dx, T1_B[1]+dy, T1_B[2]+dz, pose1[3], pose1[4], pose1[5]]
            
            zt2, d_zt2 = ic2.ordinaryCompute(zt2, d_zt2, fz2, 5)
            dx, dy, dz = R_B2T2 @ np.array([0,0,zt2])
            pose2 = [T2_B[0]+dx, T2_B[1]+dy, T2_B[2]+dz, pose2[3], pose2[4], pose2[5]]
        
        robot1_pos = ur1.servoCartesian(pose1)
        robot2_pos = ur2.servoCartesian(pose2)
        robot1_joint_recorder.writerow([robot1_pos[0],robot1_pos[1],robot1_pos[2],robot1_pos[3],robot1_pos[4],robot1_pos[5]])
        robot2_joint_recorder.writerow([robot2_pos[0],robot2_pos[1],robot2_pos[2],robot2_pos[3],robot2_pos[4],robot2_pos[5],])
        sleep(0.04)
    dualrobot_f.close()
    # robot2_f.close()
    robot1_joint_traj.close()
    robot2_joint_traj.close()
    # if forceControl1 or forceControl2:
    #     plt.plot(np.linspace(0, 0.04*len(Fz2), len(Fz2)), Fz2)
    #     plt.grid()
    #     plt.show()
    
if __name__ == '__main__':
    # six_axes_test()
    sleep(3)
    np.set_printoptions(precision = 5, suppress=True)
    main()
    # main_ros()
    
