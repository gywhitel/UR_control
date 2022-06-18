import numpy as np
from math import sin
from math import cos
from math import pi

def fk_UR5(theta):

#     d1 = 89.159
#     d4 = 109.15
#     d5 = 94.65
#     d6 = 82.3
#     a2 = -425
#     a3 = -392.25

    d1 =    89.2 / 1000.0
    d4 =   109.3 / 1000.0
    d5 =   94.75 / 1000.0
    d6 =    82.5 / 1000.0
    a2 =    -425 / 1000.0
    a3 = -392.25 / 1000.0

    T01=np.empty((4,4))
    T12=np.empty((4,4))
    T23=np.empty((4,4))
    T34=np.empty((4,4))
    T45=np.empty((4,4))
    T56=np.empty((4,4))

    p=np.matrix('0 0 0 1')

    T01[0,0]=cos(theta[0]); T01[0,1]=0; T01[0,2]=sin(theta[0]); T01[0,3]=0
    T01[1,0]=sin(theta[0]); T01[1,1]=0; T01[1,2]=-cos(theta[0]); T01[1,3]=0
    T01[2,0]=0; T01[2,1]=1; T01[2,2]=0; T01[2,3]=d1
    T01[3,:]=p

    T12[0,0]=cos(theta[1]); T12[0,1]=-sin(theta[1]); T12[0,2]=0; T12[0,3]=a2*cos(theta[1])
    T12[1,0]=sin(theta[1]); T12[1,1]=cos(theta[1]); T12[1,2]=0; T12[1,3]=a2*sin(theta[1])
    T12[2,0]=0; T12[2,1]=0; T12[2,2]=1; T12[2,3]=0
    T12[3,:]=p

    T23[0,0]=cos(theta[2]); T23[0,1]=-sin(theta[2]); T23[0,2]=0; T23[0,3]=a3*cos(theta[2])
    T23[1,0]=sin(theta[2]); T23[1,1]=cos(theta[2]); T23[1,2]=0; T23[1,3]=a3*sin(theta[2])
    T23[2,0]=0; T23[2,1]=0; T23[2,2]=1; T23[2,3]=0
    T23[3,:]=p

    T34[0,0]=cos(theta[3]); T34[0,1]=0; T34[0,2]=sin(theta[3]); T34[0,3]=0
    T34[1,0]=sin(theta[3]); T34[1,1]=0; T34[1,2]=-cos(theta[3]); T34[1,3]=0
    T34[2,0]=0; T34[2,1]=1; T34[2,2]=0; T34[2,3]=d4
    T34[3,:]=p

    T45[0,0]=cos(theta[4]); T45[0,1]=0; T45[0,2]=-sin(theta[4]); T45[0,3]=0
    T45[1,0]=sin(theta[4]); T45[1,1]=0; T45[1,2]=cos(theta[4]); T45[1,3]=0
    T45[2,0]=0; T45[2,1]=-1; T45[2,2]=0; T45[2,3]=d5
    T45[3,:]=p

    T56[0,0]=cos(theta[5]); T56[0,1]=-sin(theta[5]); T56[0,2]=0; T56[0,3]=0
    T56[1,0]=sin(theta[5]); T56[1,1]=cos(theta[5]); T56[1,2]=0; T56[1,3]=0
    T56[2,0]=0; T56[2,1]=0; T56[2,2]=1; T56[2,3]=d6
    T56[3,:]=p

    # T01 = np.matrix('cos(theta[0]) 0  sin(theta[0]) 0;\
    #                  sin(theta[0]) 0 -cos(theta[0]) 0;\
    #                   0            1           0   d1;\
    #                   0            0           0   1')
    
    # T12 = np.matrix('cos(theta[1]) -sin(theta[1])  0  a2*cos(theta[1]);\
    #                  sin(theta[1])  cos(theta[1])  0  a2*sin(theta[1]);\
    #                     0              0           1            0;\
    #                     0              0           0            1')
    
    # T23 = np.matrix('cos(theta[2]) -sin(theta[2])  0  a3*cos(theta[2]);\
    #                  sin(theta[2])  cos(theta[2])  0  a3*sin(theta[2]);\
    #                     0              0           1            0;\
    #                     0              0           0            1')
      
    # T34 = np.matrix('cos(theta[3]) 0 sin(theta[3])  0;\
    #                  sin(theta[3]) 0 -cos(theta[3]) 0;\
    #                   0            1        0      d4;\
    #                   0            0        0       1') 
    
    # T45 = np.matrix('cos(theta[4]) 0 -sin(theta[4])  0;\
    #                  sin(theta[4]) 0  cos(theta[4])  0;\
    #                   0           -1        0       d5;\
    #                   0            0        0       1')
   
    # T56 = np.matrix('cos(theta[5]) -sin(theta[5])   0 0;\
    #                  sin(theta[5]) cos(theta[5])    0 0;\
    #                    0                 0          1 d6;\
    #                    0                 0          0 1')

    T02 =  np.matmul(T01,T12)
    T03 =  np.matmul(T02,T23)
    T04 =  np.matmul(T03,T34)
    T05 =  np.matmul(T04,T45)
    T06 =  np.matmul(T05,T56)

    return T06 

if __name__=="__main__":
    # theta=np.array([42.55, -87.72, 67.21, -66.46, -90.22, -1.55]) / 180 * np.pi
    # theta=np.array([31.95, -113.24, 98.53, -165.54, -71.65, -0.86]) / 180 * np.pi
    theta=np.array([33.13, -83.52, 73.65, -170.19, -73.11, -0.63]) / 180 * np.pi
    T=fk_UR5(theta)
    print(T)
