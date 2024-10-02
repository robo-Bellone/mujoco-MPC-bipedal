from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from src.can_mit import TMotorManager_mit_can
from matplotlib.animation import FuncAnimation as plt


ID_1 = 2
ID_2 = 3

Type_1 = 'AK60-6'
Type_2 = 'AK60-6'


def two_DOF(dev3):
    # dev1.set_impedance_gains_real_unit(K=10,B=1)
    # time.sleep(1)
    # dev2.set_impedance_gains_real_unit(K=10,B=1)
    # time.sleep(1)
    dev3.set_impedance_gains_real_unit(K=10,B=1)
    time.sleep(1)

   
    loop = SoftRealtimeLoop(dt = 0.001, report=True, fade=0)
    for t in loop:
        # dev1.update()
        # dev2.update()
        dev3.update()
        if t < 1.0:
            # dev1.position = 0.0
            # dev2.position = 0.0
            dev3.position = 0.0
        else:
            # dev1.position = 0.4*np.sin(np.pi*t)
            # dev2.position = 0.4*np.sin(np.pi*t)
            dev3.position = 0.4*np.sin(np.pi*t)
        # print(dev1.position)
        # print(dev2.position)
        print(dev3.position)
    del loop


if __name__ == '__main__':
    # to use additional motors, simply add another with block
    # remember to give each motor a different log name!
   # with TMotorManager_mit_can(motor_type=Type_1, motor_ID=3) as dev1:
    #    with TMotorManager_mit_can(motor_type=Type_2, motor_ID=4) as dev2:
            with TMotorManager_mit_can(motor_type=Type_2, motor_ID=4) as dev3:
                dev3.set_zero_position()
                time.sleep(2)
                
                two_DOF(dev3)