import numpy as np
from math import sin
from math import cos
from math import atan2
from math import sqrt
from math import pi

# TOOL_H = 0.085    # without force sensor
TOOL_H = 0.105  # with force sensor mounted
TOOL_H = 0  # with force sensor mounted
INF = np.inf

def angleRange(q):
    '''
    regualte the joint angle in [-pi, pi]
    '''
    while True:
        if q <= pi and q >= -pi:
            break
        if q > pi:
            q = q - 2*pi
        if q < -pi:
            q = q + 2*pi
    return q

def norm(vec):
    '''
    brief
    Parameters
    ---
    vec : list-like
        introduction
    Return
    ---
    return the squared L2 norm of the vector
    '''
    sum = 0.0
    for element in vec:
        sum = sum + element*element
    return sum

def ik_UR5(p):
    '''
    p is x,y,z and euler angles R,P,Y\\
    Notice UR teach pandent shows rotation vector\\
    unit: m, rad
    '''
    # NOTE In python2 any number/int = int ! You have to make the denominator float !
    d1 =    89.2 / 1000.0
    d4 =   109.3 / 1000.0
    d5 =   94.75 / 1000.0
    d6 =    82.5 / 1000.0
    a2 =    -425 / 1000.0
    a3 = -392.25 / 1000.0
 
    # d1 = 89.159 mm 
    # d4 = 109.15 mm
    # d5 = 94.65 mm
    # d6 = 82.3 mm
    # a2 = -425 mm
    # a3 = -392.25 mm
    '''
    | nx ox ax px |
    | ny oy ay py |
    | nz oz az pz |
    | 0  0  0  1  |
    '''
    # Angles are given in Euler angle 
    # rotation sequence: yaw(z) -> pitch(y) -> roll(x)  
    # roll: p[3], pitch: p[4], yaw: p[5]

    Cy, Sy = cos(p[5]), sin(p[5])   # yaw
    Cp, Sp = cos(p[4]), sin(p[4])   # pitch
    Cr, Sr = cos(p[3]), sin(p[3])   # roll

    nx = Cy * Cp
    ny = Cp * Sy
    nz = -Sp
    ox = Cy * Sp * Sr - Cr * Sy
    oy = Cy * Cr + Sy * Sp * Sr
    oz = Cp * Sr
    ax = Sy * Sr + Cy * Cr * Sp
    ay = Cr * Sy * Sp - Cy * Sr
    az = Cp * Cr

    px = p[0] - ax * TOOL_H
    py = p[1] - ay * TOOL_H
    pz = p[2] - az * TOOL_H

    sym=np.matrix('1 1 1; 1 1 -1; 1 -1 1; 1 -1 -1; -1 1 1; -1 1 -1; -1 -1 1; -1 -1 -1')

    theta=np.empty((8,6))

    NUM=0
    for i in range(8):
    
        theta1 = atan2(d6*ay-py,d6*ax-px) - atan2(d4,sym[i,0]*sqrt(pow(d6*ax-px,2) + pow(d6*ay-py,2) - d4*d4))
        theta1 = angleRange(theta1)

        sin5=sym[i,1]*sqrt(1-pow(ax*sin(theta1)-ay*cos(theta1),2))

        theta5 = atan2(sin5,sin(theta1) * ax - cos(theta1) * ay)
        theta5 = angleRange(theta5)

        theta6 = atan2( (-sin(theta1) * ox + cos(theta1) * oy)/sin5,\
                     (sin(theta1) * nx - cos(theta1) * ny )/sin5)
        
        A=d5*(sin(theta6)*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6)*(ox*cos(theta1)+oy*sin(theta1)))\
                -d6*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1)

        B=pz-d1-az*d6+d5*(oz*cos(theta6)+nz*sin(theta6))

        theta3=atan2(sym[i,2]*sqrt(4*a2*a2*a3*a3-pow(A*A+B*B-a2*a2-a3*a3,2)), A*A+B*B-a2*a2-a3*a3)
        theta3 = angleRange(theta3)

        theta2=atan2((a3*cos(theta3)+a2)*B-a3*sin(theta3)*A,(a3*cos(theta3)+a2)*A+a3*sin(theta3)*B)
        theta2 = angleRange(theta2)

        C=(nx*cos(theta1)+ny*sin(theta1))*cos(theta6)*cos(theta5)-(ox*cos(theta1)+oy*sin(theta1))*sin(theta6)*cos(theta5)\
                -(ax*cos(theta1)+ay*sin(theta1))*sin(theta5)

        D=nz*cos(theta6)*cos(theta5)-oz*sin(theta6)*cos(theta5)-az*sin(theta5)

        theta4=atan2(D,C)-theta2-theta3
        theta4 = angleRange(theta4)

        theta[NUM,:] = [theta1,theta2,theta3,theta4,theta5,theta6]

        NUM=NUM+1

    return theta

def IK_optimal(targetPose, q_last=np.array([0,0,0,0,0,0])):
    '''
    brief
    Parameters
    ---
    q_last : 1x6 numpy array
        joint angle
    targetPose : 1 x 6 numpy array
        pose (x,y,z,roll,pitch,yaw), rotation order: Z -> Y -> X. 
        Unit: m, rad
    Return : 1 x 6 numpy array
    ---
    return a set of solution closest to q_last
    '''
    optimalCost = INF
    optimal = np.zeros((1,6))
    solution = ik_UR5(targetPose)
    for group in solution:
        cost = norm(group - q_last) + 5*abs(group[0]-q_last[0]) + 5*abs(group[1]-q_last[1])
        if optimalCost > cost:
            optimalCost = cost
            optimal = group
    return optimal

#p is x,y,z and euler angles R,P,Y
def ik_UR5_XYZ(p):
    '''
    p is x,y,z and euler angles R,P,Y\\
    Notice UR teach pandent shows rotation vector\\
    unit: m, rad
    '''
    # NOTE In python2 any number/int = int ! You have to make the denominator float !
    d1 =    89.2 / 1000.0
    d4 =   109.3 / 1000.0
    d5 =   94.75 / 1000.0
    d6 =    82.5 / 1000.0
    a2 =    -425 / 1000.0
    a3 = -392.25 / 1000.0
 
    # d1 = 89.159 mm 
    # d4 = 109.15 mm
    # d5 = 94.65 mm
    # d6 = 82.3 mm
    # a2 = -425 mm
    # a3 = -392.25 mm
    '''
    | nx ox ax px |
    | ny oy ay py |
    | nz oz az pz |
    | 0  0  0  1  |
    '''
    # Angles are given in Euler angle 
    # rotation sequence: roll(x) -> pitch(y) -> yaw(z)
    # roll: p[3], pitch: p[4], yaw: p[5]
    # ? Incorrect rotation order ? Different from mainstream Euler Angle yaw -> pitch -> roll
    nx=cos(p[4])*cos(p[5])
    ny=sin(p[3])*sin(p[4])*cos(p[5])+cos(p[3])*sin(p[5])
    nz=-cos(p[3])*sin(p[4])*cos(p[5])+sin(p[3])*sin(p[5])

    ox=-cos(p[4])*sin(p[5])
    oy=-sin(p[3])*sin(p[4])*sin(p[5])+cos(p[3])*cos(p[5])
    oz=cos(p[3])*sin(p[4])*sin(p[5])+sin(p[3])*cos(p[5])

    ax=sin(p[4])
    ay=-sin(p[3])*cos(p[4])
    az=cos(p[3])*cos(p[4])

    px = p[0] - ax * TOOL_H
    py = p[1] - ay * TOOL_H
    pz = p[2] - az * TOOL_H

    sym=np.matrix('1 1 1; 1 1 -1; 1 -1 1; 1 -1 -1; -1 1 1; -1 1 -1; -1 -1 1; -1 -1 -1')

    theta=np.empty((8,6))

    NUM=0
    for i in range(8):
    
        theta1 = atan2(d6*ay-py,d6*ax-px) - atan2(d4,sym[i,0]*sqrt(pow(d6*ax-px,2) + pow(d6*ay-py,2) - d4*d4))
        theta1 = angleRange(theta1)

        sin5=sym[i,1]*sqrt(1-pow(ax*sin(theta1)-ay*cos(theta1),2))

        theta5 = atan2(sin5,sin(theta1) * ax - cos(theta1) * ay)
        theta5 = angleRange(theta5)

        theta6 = atan2( (-sin(theta1) * ox + cos(theta1) * oy)/sin5,\
                     (sin(theta1) * nx - cos(theta1) * ny )/sin5)
        
        A=d5*(sin(theta6)*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6)*(ox*cos(theta1)+oy*sin(theta1)))\
                -d6*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1)

        B=pz-d1-az*d6+d5*(oz*cos(theta6)+nz*sin(theta6))

        theta3=atan2(sym[i,2]*sqrt(4*a2*a2*a3*a3-pow(A*A+B*B-a2*a2-a3*a3,2)), A*A+B*B-a2*a2-a3*a3)
        theta3 = angleRange(theta3)

        theta2=atan2((a3*cos(theta3)+a2)*B-a3*sin(theta3)*A,(a3*cos(theta3)+a2)*A+a3*sin(theta3)*B)
        theta2 = angleRange(theta2)

        C=(nx*cos(theta1)+ny*sin(theta1))*cos(theta6)*cos(theta5)-(ox*cos(theta1)+oy*sin(theta1))*sin(theta6)*cos(theta5)\
                -(ax*cos(theta1)+ay*sin(theta1))*sin(theta5)

        D=nz*cos(theta6)*cos(theta5)-oz*sin(theta6)*cos(theta5)-az*sin(theta5)

        theta4=atan2(D,C)-theta2-theta3
        theta4 = angleRange(theta4)

        theta[NUM,:] = [theta1,theta2,theta3,theta4,theta5,theta6]

        NUM=NUM+1

    return theta


def ik_UR_mat(T):

    d1 =    89.2 / 1000
    d4 =   109.3 / 1000
    d5 =   94.75 / 1000
    d6 =    82.5 / 1000
    a2 =    -425 / 1000
    a3 = -392.25 / 1000

    # d1 = 89.159
    # d4 = 109.15
    # d5 = 94.65
    # d6 = 82.3
    # a2 = -425
    # a3 = -392.25

    nx = T[0,0]; ny = T[1,0]; nz = T[2,0]
    ox = T[0,1]; oy = T[1,1]; oz = T[2,1]
    ax = T[0,2]; ay = T[1,2]; az = T[2,2]
    px = T[0,3]; py = T[1,3]; pz = T[2,3]

    sym=np.matrix('1 1 1;1 1 -1;1 -1 1;1 -1 -1;-1 1 1;-1 1 -1;-1 -1 1;-1 -1 -1')

    theta=np.empty((8,6))

    NUM=0
    for i in range(8):
    
        theta1 = atan2(d6*ay-py,d6*ax-px) - atan2(d4,sym[i,0]*sqrt(pow(d6*ax-px,2) + pow(d6*ay-py,2) - d4*d4))

        sin5=sym[i,1]*sqrt(1-pow(ax*sin(theta1)-ay*cos(theta1),2))

        theta5 = atan2(sin5,sin(theta1) * ax - cos(theta1) * ay)

        theta6 = atan2( (-sin(theta1) * ox + cos(theta1) * oy)/sin5,\
                     (sin(theta1) * nx - cos(theta1) * ny )/sin5)

        A=d5*(sin(theta6)*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6)*(ox*cos(theta1)+oy*sin(theta1)))\
                -d6*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1)

        B=pz-d1-az*d6+d5*(oz*cos(theta6)+nz*sin(theta6))

        theta3=atan2(sym[i,2]*sqrt(4*a2*a2*a3*a3-pow(A*A+B*B-a2*a2-a3*a3,2)),A*A+B*B-a2*a2-a3*a3)

        theta2=atan2((a3*cos(theta3)+a2)*B-a3*sin(theta3)*A,(a3*cos(theta3)+a2)*A+a3*sin(theta3)*B)

        C=(nx*cos(theta1)+ny*sin(theta1))*cos(theta6)*cos(theta5)-(ox*cos(theta1)+oy*sin(theta1))*sin(theta6)*cos(theta5)\
                -(ax*cos(theta1)+ay*sin(theta1))*sin(theta5)

        D=nz*cos(theta6)*cos(theta5)-oz*sin(theta6)*cos(theta5)-az*sin(theta5)

        theta4=atan2(D,C)-theta2-theta3

        theta[NUM,:] = [theta1,theta2,theta3,theta4,theta5,theta6]

        NUM=NUM+1

    return theta

if __name__ == '__main__':
    for i in range(5):
        pose = q = np.array([0,0,0,0,0,0])
        pose[0:3] = np.random.rand(3)
        pose[3:] = 2*(np.random.rand(1)-0.5)*pi
        q = IK_optimal(pose, q)
        print(q)
