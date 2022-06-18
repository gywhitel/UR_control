#coding=utf-8

from __future__ import print_function
from logging import root
import socket
import struct
import time
from dual_arm_circle import PI
import ik_UR5
import numpy as np
import threading
import sys
from fk_UR5 import fk_UR5
# from forceControl import ForceSensor, sensor_connection_thread

pi = np.pi

def str2byte(data):
    '''
    transform str to bytes
    '''
    # return bytes(data, encoding='utf-8')
    return bytes(data.encode('utf-8'))



class UR_state:

    def __init__(self):
        self.__HOST = '192.168.0.3'
        self.__PORT = 30003			
        
    def setIP(self, ip):
        """
        Default IP is '192.168.0.2' 
        """
        self.__HOST = ip

    def connect(self):
        '''
        connect UR
        seperate UR connection from constructor
        '''
        self.__robot = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.__robot.settimeout(2.0)
        self.__robot.connect((self.__HOST,self.__PORT))
        print("Successful connection to UR", self.__HOST)


    def getState(self):
        # return bytearray(self.__robot.recv(1108))
        self.__msg = bytearray(self.__robot.recv(1108))

    def getJointPosition(self):
        '''
        read joint angles in rad
        '''
        # recvData = self.getMessage()
        # read joint angles
        self.q1 = struct.unpack('!d', self.__msg[252:260])[0]
        self.q2 = struct.unpack('!d', self.__msg[260:268])[0]
        self.q3 = struct.unpack('!d', self.__msg[268:276])[0]
        self.q4 = struct.unpack('!d', self.__msg[276:284])[0]
        self.q5 = struct.unpack('!d', self.__msg[284:292])[0]
        self.q6 = struct.unpack('!d', self.__msg[292:300])[0]
        return [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6]

    def getJointVelocity(self):
        # recvData = self.getMessage()
        # read joint angle velocity
        self.qd2 = struct.unpack('!d', self.__msg[308:316])[0]
        self.qd1 = struct.unpack('!d', self.__msg[300:308])[0]
        self.qd3 = struct.unpack('!d', self.__msg[316:324])[0]
        self.qd4 = struct.unpack('!d', self.__msg[324:332])[0]
        self.qd5 = struct.unpack('!d', self.__msg[332:340])[0]
        self.qd6 = struct.unpack('!d', self.__msg[340:348])[0]
        return [self.qd1, self.qd2, self.qd3, self.qd4, self.qd5, self.qd6]
             
    def securityConstraint(self, q, limit = 0.08, step=True):
        '''
        verify whether commanded configuration is too far from current configuration.
        return True if not exceed security limit.
        '''
        jointAngle = self.getJointPosition()
        # Axis 0 limit
        if abs(q[0] - jointAngle[0]) > pi:
            print('Axis 0 is about to rotate over 180 degrees. Abort operation!')
            raise("Dangerous operation.")
            return False
        elif step:
            if abs(q[0] - jointAngle[0]) > pi/2 or abs(q[1] - jointAngle[1]) > limit or abs(q[2] - jointAngle[2]) > limit or abs(q[3] - jointAngle[3]) > limit or abs(q[4] - jointAngle[4]) > limit or abs(q[5] - jointAngle[5]) > limit+ pi/2:
                flag = sys.stdin.readline('Beyond safety distance! Please confirm, yes/no: ')
                if flag == 'yes':
                    return True
                else:
                    print('STOP!')
                    return False
            else:
                return True
        else:
            return True

    def get_TCP_pose(self):
        '''
        return xyz and rx, ry, rz of TCP \\
        NOTE: this is rotation vector, not Euler angle
        # NOTE: rpy Euler angel, but slightly different from readings on the pendant
        unit: m , rad
        '''
        # recvData = self.getMessage()
        # the pose of tcp
        self.x = struct.unpack('!d', self.__msg[444:452])[0]
        self.y = struct.unpack('!d', self.__msg[452:460])[0]
        self.z = struct.unpack('!d', self.__msg[460:468])[0]
        self.rx = struct.unpack('!d', self.__msg[468:476])[0]
        self.ry = struct.unpack('!d', self.__msg[476:484])[0]
        self.rz = struct.unpack('!d', self.__msg[484:492])[0]
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    def get_TCP_velocity(self):
        # the velocity of tcp
        # recvData = self.getMessage()
        self.vx = struct.unpack('!d', self.__msg[492:500])[0]
        self.vy = struct.unpack('!d', self.__msg[500:508])[0]
        self.vz = struct.unpack('!d', self.__msg[508:516])[0]
        self.v_rx = struct.unpack('!d', self.__msg[516:524])[0]
        self.v_ry = struct.unpack('!d', self.__msg[524:532])[0]
        self.v_rz = struct.unpack('!d', self.__msg[532:540])[0]
        return [self.vx, self.vy, self.vz, self.v_rx, self.v_ry, self.v_rz]

    def get_TCP_force(self):
        # recvData = self.getMessage()
        # the force of tcp
        self.Fx = struct.unpack('!d', self.__msg[540:548])[0]
        self.Fy = struct.unpack('!d', self.__msg[548:556])[0]
        self.Fz = struct.unpack('!d', self.__msg[556:564])[0]
        self.Tx = struct.unpack('!d', self.__msg[564:572])[0]
        self.Ty = struct.unpack('!d', self.__msg[572:580])[0]
        self.Tz = struct.unpack('!d', self.__msg[580:588])[0]
        return [self.Fx, self.Fy, self.Fz, self.Tx, self.Ty, self.Tz]

    def movej(self, q):
        '''
        move to certain position (in joint space)\\
        [In] q: list of joint angles  
        移动到某一位置（关节空间内线性)  
        '''
        data ='movej('+str(q)+',a=0.5,v=0.3)\n'
        data = str2byte(data)
        self.__robot.send(data)

    def stopj(self, a=0.5):
        '''
        停止(关节空间内线性),将关节速度减至0  
        [in] a: 关节加速度  
        '''
        data ='stopj('+str(a)+')\n'
        data = str2byte(data)
        self.__robot.send(data)
    
    def servoj(self, q):
        '''
        '''
        # instruction = "servoj(" + str(q) + ",0,0,0.06,0.03,300)\n"

        instruction = "servoj(" + str(q) + ",0,0,0.06,0.03,300)\n"
        instruction = str2byte(instruction)
        self.__robot.send(instruction)

    
    def moveCartesian(self, p, set=0):
        '''
        Move to a point p specified in Cartesian space   
        @param p: TCP pose [x, y, z, rx, ry, rz] /m, rad
        '''
        
        # theta=np.empty((8,6))   # 8 sets of solution
        # theta[:,:] = 
        q1 = ik_UR5.ik_UR5(p)[set]        # take 3rd set of solution here
        q=[q1[0],q1[1],q1[2],q1[3],q1[4],q1[5]]
        print("joint angle:", q)
        # fk_pose = fk_UR5(q)
        # print('Target pose:[', fk_pose[0,3],fk_pose[1,3],fk_pose[2,3], "]")
        # security constraint
        if self.securityConstraint(q, 0.2, False):
            self.movej(q)

    def moveCartesianOptimal(self, pose):
        '''
        select one group optimal inverse kinematic solution based on current configuration
        '''
        self.jointAngle = ik_UR5.IK_optimal(pose, self.jointAngle)
        print(self.jointAngle)
        q = [0,0,0,0,0,0]
        for i in range(6):
            q[i] = self.jointAngle[i]
        if self.securityConstraint(q, 0.2, False):
            self.movej(q) 

    def servoCartesian(self, p, set=0):
        '''
        Move to a point p specified in Cartesian space   
        @param p: TCP pose [x, y, z, rx, ry, rz] /m, rad
        '''
        
        # theta=np.empty((8,6))   # 8 sets of solution
        # theta[:,:] = 
        q1 = ik_UR5.ik_UR5(p)[set]
        q=[q1[0],q1[1],q1[2],q1[3],q1[4],q1[5]]
        # Axis 6 singularity
        if (q[5] - self.jointAngle[5]) > 0.9*2*PI:
            q[5] -= 2*PI
        if (q[5] - self.jointAngle[5]) < -0.9*2*PI:
            q[5] += 2*PI
        # print(q)
        if self.securityConstraint(q, 0.2, True):
            self.servoj(q)
            self.jointAngle = q
        return q
    
    def set_initial_configuration(self, q):
        '''
        Set initial configuration 
        '''
        self.jointAngle = q

    def servoCartesianOptimal(self, pose):
        '''
        select one group optimal inverse kinematic solution based on current configuration
        '''
        self.jointAngle = ik_UR5.IK_optimal(pose, self.jointAngle)
        q = [0,0,0,0,0,0]
        for i in range(6):
            q[i] = self.jointAngle[i]
        print(q)
        if self.securityConstraint(q, 0.2, True):
            self.servoj(q)        


class ur_connection_thread(threading.Thread):

    def __init__(self, name, robot):
        threading.Thread.__init__(self)
        self.name = name
        self.__robot = robot
    
    def run(self):
        while True:
            self.__robot.getState()

if __name__=="__main__":

    ur = UR_state()
    ur.setIP('192.168.0.2')
    ur.connect()
    ur.getState()

    ur_read = ur_connection_thread('ur_read', ur)
    ur_read.setDaemon(True)
    ur_read.start()

    jointAngle = ur.getJointPosition()
    print("Initial joint angle:", jointAngle)
    print("Initial pose:",ur.get_TCP_pose())
    ur.set_initial_configuration(jointAngle)

    forceSensor = ForceSensor('192.168.0.200')
    forceSensor.tare()
    forceSensor.start()

    sensor_connect = sensor_connection_thread('sensor_connect', forceSensor)
    sensor_connect.setDaemon(True)
    sensor_connect.start()

    F = []
    initialPose = [0.184, 0.476, 0.213, pi, 0, 0]    # sanding pad center

    for step in np.linspace(0, -0.009, 100):
        pose = initialPose
        pose[2] = pose[2] + step
        # ur.servoCartesianOptimal(pose)
        F.append(forceSensor.F[2])  
        time.sleep(0.04)

    # ur.moveCartesianOptimal(pose)
