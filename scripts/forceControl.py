from math import pi
import numpy as np
import struct
import socket
import threading
from time import sleep
from ik_UR5 import IK_optimal
import matplotlib.pyplot as plt

sin = np.sin
cos = np.cos

def pose_to_transMat(pose):
        '''
        Return current a transformation matrix from robot base to (UR TCP). 
        Transform pose into a 4x4 transformation matrix.
        Parameter
        ---
        6x1 list (x, y, z, roll, pitch, yaw)  (m, rad)
        Return
        ---
        return a 4x4 homogeneous transformation matrix (4x4 np.array)
        '''
        Cy, Sy = cos(pose[5]), sin(pose[5])   # yaw
        Cp, Sp = cos(pose[4]), sin(pose[4])   # pitch
        Cr, Sr = cos(pose[3]), sin(pose[3])   # roll

        nx = Cy * Cp
        ny = Cp * Sy
        nz = -Sp
        ox = Cy * Sp * Sr - Cr * Sy
        oy = Cy * Cr + Sy * Sp * Sr
        oz = Cp * Sr
        ax = Sy * Sr + Cy * Cr * Sp
        ay = Cr * Sy * Sp - Cy * Sr
        az = Cp * Cr

        px = pose[0]
        py = pose[1]
        pz = pose[2]
        
        return np.array([[nx, ox, ax, px],
                         [ny, oy, ay, py],
                         [nz, oz, az, pz],
                         [0,  0,  0,  1]])

def pose_to_rotMat(pose):
    '''
    Return current a transformation matrix from robot base to (UR TCP). 
    Transform pose into a 4x4 transformation matrix.
    Parameter
    ---
    6x1 list (x, y, z, roll, pitch, yaw)  (m, rad)
    Return
    ---
    return a 3x3 rotation matrix (3x3 np.array)
    '''
    Cy, Sy = cos(pose[5]), sin(pose[5])   # yaw
    Cp, Sp = cos(pose[4]), sin(pose[4])   # pitch
    Cr, Sr = cos(pose[3]), sin(pose[3])   # roll

    nx = Cy * Cp
    ny = Cp * Sy
    nz = -Sp
    ox = Cy * Sp * Sr - Cr * Sy
    oy = Cy * Cr + Sy * Sp * Sr
    oz = Cp * Sr
    ax = Sy * Sr + Cy * Cr * Sp
    ay = Cr * Sy * Sp - Cy * Sr
    az = Cp * Cr
    return np.array([[nx, ox, ax],
                        [ny, oy, ay],
                        [nz, oz, az]])

class ImpedanceController:

    T = 0.002   # sampling period
    # Uppercase indicates vectors or matrices
    X_d = [0,0,0]   # [x_d, y_d, z_d]
    dP = [0,0,0]     # modified position (deltaPosition )in {T} [x, y, z]
    

    def __init__(self, m=1, b=100, k=0):
        self.M = m
        self.B = b
        self.K = k
    
    def setDesiredForce(self, fx, fy, fz):
        self.Fd = [fx, fy, fz]    # desired force [Fd_x, Fd_y, Fd_z]

    def setSamplingTime(self, t):
        self.T = t
   
    def ordinaryCompute(self, x, x_d, f, Fd):
        '''
        One-cartesian-dimensional ordinary impedance controller.
        M, B, K should be 6x6 diagonal matrices. They are constant here for simplicity (the same on each dimension).
        Parameters
        ---
        x : float
          position, one element of pose vector
        x_d : float
            velocity, coming from last iteration
        f : float
          force read from force sensor
        Fd : float 
            desired force
        '''
        x_dd = (1/self.M) * (Fd - f - self.B * x_d - self.K * x)
        x_d = x_d + x_dd * self.T
        x = x + x_d * self.T
        # x = self.x_d * self.T
        return [x, x_d]
    
    def ordinaryCompute2(self, x, x_d, f, Fd, M, B, K):
        '''
        One-cartesian-dimensional ordinary impedance controller.
        M, B, K should be 6x6 diagonal matrices. They are constant here for simplicity (the same on each dimension).
        Parameters
        ---
        x : float
          position, one element of pose vector
        x_d : float
            velocity, coming from last iteration
        f : float
          force read from force sensor
        Fd : float 
            desired force
        '''
        x_dd = (1/M) * (Fd - f - B * x_d - K * x)
        x_d = x_d + x_dd * self.T
        x = x + x_d * self.T
        # x = self.x_d * self.T
        return [x, x_d]

    def avic(self, x, x_d, f, Fd):
        gamma = 0.5 * (Fd-f)/self.B
        deltaB = self.B / (x_d + 1e-8) * gamma
        x_dd = (1/self.M) * (Fd - f - (self.B + deltaB) * x_d - self.K * x)
        x_d = x_d + x_dd * self.T
        x = x + x_d * self.T
        return [x, x_d]

    def environmentCompute(self, x, x_d, xe_d, f, Fd):
        '''
        One-cartesian-dimensional EIFIC.

        Parameters
        ---
        x : float
          one element of pose vector
        x_d : float
            velocity coming from last iteration
        xe_d : float
            environment tangent
        f : float
          force read from force sensor
        Fd : float 
            desired force

        Return
        ---
        [float]
        reference position from impedance controller
        '''
        x_dd = (1/self.M) * (Fd - f - self.B *(x_d - xe_d))
        x_d = x_d + x_dd * self.T
        x = x + x_d * self.T
        return [x, x_d]

    def environmentComputeMat(self, X, X_d, Xe_d, F, Fd):
        X_dd = np.linalg.inv(self.M) * (Fd - F - self.B * (X_d - 0.3*Xe_d))
        X_d = X_d + X_dd * self.T
        X = X + X_d * self.T

    def ordinary_impedance_3D(self, pose2, F, axis):
        '''
        pose : 1 x 6 list
            next theoretical trajectory point
        axis : list
            [0(x),1(y),2(z)]
        '''
        for i in axis:     # x, z direction
            self.dP[i], self.X_d[i] = self.ordinaryCompute(self.dP[i], self.X_d[i], F[i], self.Fd[i], self.M[i], self.B[i], self.K[i])
        T_bt = pose_to_transMat(pose2)
        contactPoint = np.dot(T_bt, np.array([self.dP[0], self.dP[1], self.dP[2], 1]))
        poseMod = [contactPoint[0], contactPoint[1], contactPoint[2], pose2[3], pose2[4], pose2[5]]
        print(np.array(poseMod[:3]) - np.array(pose2[:3]))
        return poseMod

    def ordinary_impedance_Z(self, pose2, fz):
        '''
        Force tracking along Z-axis of tool frame {T}
        '''
        # self.dP[3] = self.ordinaryCompute(self.delta_Z, fz)  # delta Z under {T}
        # T_bt = pose_to_transMat(pose2)      # change reference frame from {T} to {B}
        # delta_P = np.dot(T_bt , np.array([0, 0, self.delta_Z, 1]))   # transform to {B}
        # pose = [delta_P[0], delta_P[1], delta_P[2], pose2[3], pose2[4], pose2[5]]
        # print(np.array(pose[:3]) - np.array(pose2[:3]))
        # return pose
        return self.ordinary_impedance_3D(pose2, [0,0,fz], [2])

    def ordinay_impedance_XZ(self, pose2, F):
        '''
        wrapper of 3D
        '''
        return self.ordinary_impedance_3D(pose2, F, [0,2])

    def ordinay_impedance_YZ(self, pose2, F):
        '''
        wrapper of 3D
        '''
        return self.ordinary_impedance_3D(pose2, F, [1,2])

    def setXE0(self, pose):
        '''
        Parameter
        ---
        (x, y, z, roll, pitch, yaw)
        '''
        # self.endPose = pose
        self.XE = pose

    def EIFIC_3D(self, nextPose, F, axis):
        '''
        nextPose : 1 x 6 list
        F : 1 x 3 list
        axis : list
            [0(x),1(y),2(z)]
        '''
        x1, y1, z1 = self.XE[:3]  # current pose
        x2, y2, z2 = nextPose[:3]  # next pose
        # Xe_dot in {B}
        Xed_b = np.array([(x2 - x1) / self.T, (y2 - y1) / self.T, (z2 - z1) / self.T])
        R_bt = pose_to_rotMat(self.XE) # Xe_i or real pose ?
        R_tb = R_bt.transpose()
        Xe_d = list(np.dot(R_tb, Xed_b)) # Xed_t
        # print(Xe_d)

        self.XE = nextPose
        for i in axis:
            self.dP[i], self.X_d[i] = self.environmentCompute(self.dP[i], self.X_d[i], Xe_d[i], F[i], self.Fd[i])
        
        T_bt = pose_to_transMat(nextPose)
        contactPoint = np.dot(T_bt, np.array([self.dP[0], self.dP[1], self.dP[2], 1]))
        poseMod = [contactPoint[0], contactPoint[1], contactPoint[2], nextPose[3], nextPose[4], nextPose[5]]
        print(poseMod[:3] - np.array([x2, y2, z2]))     # commanded position modified by force controller
        return poseMod

    def EIFIC_Z(self, nextPose, fz):
        '''
        Force tracking along Z-axis of tool frame {T}
        Parameters
        ---
        nextPose : 1x6 list
            next trajectory point. This is a point on the object surface, a theoretical pose.
        fz : float
          Z-axis force read from force sensor
        ---
        return reference pose computed by impedace controller under base frame {B}
        '''
        return self.EIFIC_3D(nextPose, [0,0,fz], [2])
    
    def EIFIC_XZ(self, nextPose, F):
        '''
        nextPose : 1x6 list
            next trajectory point. This is a point on the object surface, a theoretical pose.
        F : 1 x 3 list
            [Fx, Fy, Fz]
        '''
        return self.EIFIC_3D(nextPose, F, [0,2])


def printMsg(msg):
    print("Msg len: " + str(msg[0]) + " Msg type: " + str(msg[1]) + "")
    
    dataStr = "DATA: "
    for i in range(msg[0] - 2):
        dataStr += str(msg[i + 2]) + " "

    print(dataStr)


PORT = 2001

CMD_TYPE_SENSOR_TRANSMIT 	= '07'
SENSOR_TRANSMIT_TYPE_START 	= '01'
SENSOR_TRANSMIT_TYPE_STOP 	= '00'

CMD_TYPE_SET_CURRENT_TARE 	= '15'
SET_CURRENT_TARE_TYPE_NEGATIVE	= '01'

class ForceSensor:

    def __init__(self, IP):
        self.sensor = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sensor.settimeout(2.0)
        self.sensor.connect((IP, PORT))
        print('Successfully connect to Force Sensor', IP)
        self.F = [0,0,0,0,0,0]
    
    def receive(self):
        self.data =  bytearray(self.sensor.recv(2))
        #Wait until and ACK msg is send back for the stop command. (From official demo code)
        while len(self.data) < self.data[0]:
            self.data += bytearray(self.sensor.recv(self.data[0] - len(self.data)))
        # printMsg(self.data)

    def start(self):
        sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_START
        sendData = bytearray.fromhex(sendData)
        self.sensor.send(sendData)
        self.receive()
        print("Force sensor is ready to receive data.")

    def read(self):
        '''
        It should be noted that there should not be any interval between two read. Interval between two read cause incorrect data, which is historical data
        '''
        self.receive()
        self.receive()
        Fx = struct.unpack('!d', self.data[2:10])[0]
        Fy = struct.unpack('!d', self.data[10:18])[0]
        Fz = struct.unpack('!d', self.data[18:26])[0]
        Tx = struct.unpack('!d', self.data[26:34])[0]
        Ty = struct.unpack('!d', self.data[34:42])[0]
        Tz = struct.unpack('!d', self.data[42:50])[0]
        return [Fx, Fy, Fz, Tx, Ty, Tz]

    def tare(self):
        sendData = '03' + CMD_TYPE_SET_CURRENT_TARE + SET_CURRENT_TARE_TYPE_NEGATIVE
        sendData = bytearray.fromhex(sendData)
        self.sensor.send(sendData)
        print("Force sensor has been tared.")

    def close(self):
        sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_STOP
        sendData = bytearray.fromhex(sendData)
        self.sensor.send(sendData)
        self.receive()



class sensor_connection_thread(threading.Thread):

    def __init__(self, name, sensor):
        threading.Thread.__init__(self)
        self.name = name
        self.sensor = sensor
        self.denoise = False
    
    def filterON(self, filter):
        self.X = np.zeros((12,1))
        self.P = np.eye(12)
        self.denoise = True
        self.filter = filter

    def run(self):
        while True:
            self.sensor.F = self.sensor.read()
            if self.denoise:
                self.sensor.F = self.filter.filter(self.sensor.read())
                

if __name__ == '__main__':
    # ur = UR_state()
    # ur.connect()
    # ur.getState()

    # ur_connect = ur_connection_thread('ur_connect', ur)
    # ur_connect.setDaemon(True)
    # ur_connect.start()

    # initialPosition = ur.getJointPosition()
    # ur.set_initial_configuration(initialPosition)
    # print("Initial joint angle:", initialPosition)

    forceSensor = ForceSensor('192.168.0.200')
    forceSensor.tare()
    forceSensor.start()

    # kf = KalmanFilter6D()

    sensor_connect = sensor_connection_thread('sensor_connect', forceSensor)
    sensor_connect.setDaemon(True)
    # sensor_connect.filterON(kf)
    sensor_connect.start()

    # controller = ImpedanceController(1, 200, 0, 10)

    # pose = [-0.498, -0.183, 0.334, pi, 0, 0]
    # ur.moveCartesianOptimal(pose)
    # sleep(5)
    # initial = [-0.498, -0.183, 0.334, pi, 0, 0]
    # print("Good to go")

    # controller.setPose(pose)    # set starting configuration
    # q = initialPosition
  
    # gravity compensation
    # Test Kalman filter
    # MG = 1.003*9.8
    # PI = np.pi
    # dt = 0.04
    F, filteredFz = [], []
    for i in range(10000):
    # while True:
        # jointAngle = ur.getJointPosition()
        # zz = fk_UR5(jointAngle)[2,2]
        # print(pose[3:], end=';')
        # Fz = forceSensor.F[2] +  MG - MG *(-zz)
        Fz = forceSensor.F
        # Fz = -Fz
        F.append(Fz[2])  
        # fz_kf = kalmanFilter.kalman_filter_1D(Fz)
        # filteredFz.append(fz_kf)
        # print(Fz[0],Fz[2])
        # print(abs(forceSensor.F[2]), Fz)
        sleep(0.001)
    # time = np.arange(0, dt*1000, dt)
    time = np.linspace(0, len(F), len(F))
    plt.plot(time, F, label='raw')
    # plt.plot(time, filteredFz, label='filtered')
    plt.ylabel('F (N)')
    plt.xlabel('Time (s)')
    plt.legend('Fz')
    plt.grid()
    plt.show()