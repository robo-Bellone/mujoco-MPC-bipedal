from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from src.can_mit_limit import TMotorManager_mit_can
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import math
import os

ID_1 = 2
ID_2 = 3

Type_1 = 'AK60-6'
Type_2 = 'AK60-6'

err = 0.0

def tor_pd(dev1):
    dev1.set_current_gains()

    fig, ax = plt.subplots()
    line1, = ax.plot([], [], label='Device 1')
    ax.set_xlim(0, 10)  # Changed the x-axis limit to show more data over time
    ax.set_ylim(-0.8, 0.8)
    ax.set_xlabel('Time')
    ax.set_ylabel('Position')
    ax.set_title('Device Positions Over Time')
    ax.legend()
    ax.grid(True)

    t_data = []
    y_data = []

    loop = SoftRealtimeLoop(dt=0.001, report=True, fade=0)

    def init():
        line1.set_data([], [])
        return line1,

    def update3(frame):
        t = frame * loop.dt
        dev1.update()
        if t < 1.0:
            dev1.torque = 0.0
        else:
            dev1.torque = 0.5 
        

        t_data.append(t)
        y_data.append(dev1.torque)

        line1.set_data(t_data, y_data)
        ax.set_xlim(0, max(10, t))  # Update x-axis limit dynamically
        return line1,

    ani = FuncAnimation(fig, update3, frames=np.arange(0, 10000), init_func=init, blit=True, interval=1, repeat=True)
    plt.show()
    del loop

def pos_pd(dev1):
    dev1.set_impedance_gains_real_unit_full_state_feedback(K=35, B=10)   #K=68, B=8.55
    dev1.set_zero_position()
    time.sleep(2)
    fig, ax = plt.subplots()
    line1, = ax.plot([], [], label='Device 1')
    ax.set_xlim(0, 5)  # Changed the x-axis limit to show more data over time
    ax.set_ylim(-0.9, 0.9)
    ax.set_xlabel('Time')
    ax.set_ylabel('Position')
    ax.set_title('Device Positions Over Time')
    ax.legend()
    ax.grid(True)

    t_data = []
    y_data = []

   
  

    loop = SoftRealtimeLoop(dt=0.001, report=True, fade=0)

    def init():
        line1.set_data([], [])
        return line1,

    def update2(frame):
        t = frame * loop.dt
        dev1.update()

        if math.floor(t % 6) == 0 or  math.floor(t % 6) == 2 or math.floor(t % 6) == 4 :
             destination = 0.0
             a1 = -0.0
             dev1.position = destination
             dev1.torque = a1*(destination - dev1.position)
             
        elif math.floor(t % 6) == 1 :
             destination = -0.2
             dev1.position = destination
             a = 0.0
             dev1.torque = a*(destination - dev1.position)
        
        elif math.floor(t % 6) == 3 :
             destination = -0.3
             dev1.position = destination
             a = 0.0
             dev1.torque = a*(destination - dev1.position)
        elif math.floor(t % 6) == 5 :
             destination = -0.4
             dev1.position = destination
             a = 0.0
             dev1.torque = a*(destination - dev1.position)

   
        print(dev1.position)

        t_data.append(t)
        y_data.append(dev1.position)

        line1.set_data(t_data, y_data)
        ax.set_xlim(0, max(10, t))  # Update x-axis limit dynamically
        return line1,

    ani = FuncAnimation(fig, update2, frames=np.arange(0, 10000), init_func=init, blit=True, interval=1, repeat=True)
    plt.show()
    del loop

if __name__ == '__main__':
    try :
        with TMotorManager_mit_can(motor_type='gim4310', motor_ID=5) as dev1:
            pos_pd(dev1)
    
    except :
        print('down')
    finally :
        os.system( 'sudo /sbin/ip link set can0 down')        