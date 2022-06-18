import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    traj = pd.read_csv('/home/yinghao/catkin_ws/src/dualarm/data/joints_traj.csv')
    r1q1 = traj['R1J1']
    r1q2 = traj['R1J2']
    r1q3 = traj['R1J3']
    r1q4 = traj['R1J4']
    r1q5 = traj['R1J5']
    r1q6 = traj['R1J6']
    r2q1 = traj['R2J1']
    r2q2 = traj['R2J2']
    r2q3 = traj['R2J3']
    r2q4 = traj['R2J4']
    r2q5 = traj['R2J5']
    r2q6 = traj['R2J6']
    time = np.linspace(0, 50, len(r1q1))

    font1 = {'family' : 'Times New Roman',
        'weight' : 'normal',
        'size'   : 20        }

    plt.plot(time, r2q1, label='q1')
    plt.plot(time, r2q2, label='q2')
    plt.plot(time, r2q3, label='q3')
    plt.plot(time, r2q4, label='q4')
    plt.plot(time, r2q5, label='q5')
    plt.plot(time, r2q6, label='q6')
    plt.legend(prop=font1)
    plt.show()