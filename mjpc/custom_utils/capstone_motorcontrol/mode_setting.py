from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from src.can_mit import TMotorManager_mit_can
from src.motor_control import MotorControl

import time
import numpy as np
import can

Type = {
    'others':'AK60-6',
    'ankle':'gim4310'
    }

Motor = {
    'R_waist' : 2,  
    'R_hip' : 3, 
    'R_knee' : 4,  
    'R_ankle' : 5,

    'L_waist' : 6,
    'L_hip' : 7, 
    'L_knee' : 8,
    'L_ankle':9
}




def read_only(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None,mc=None):
    pos_high = [0,0,0,0,0,0,0,0]
    pos_low = [0,0,0,0,0,0,0,0]
    feedbacks = [[0 for _ in range(3)] for _ in range(8)]
    Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
    Update(L_waist,R_waist,L_hip,R_hip,L_knee,R_knee,L_ankle,R_ankle)
    for i, joint in Named:
            if joint.position > pos_high[i]:
               pos_high[i] = joint.position
            elif joint.position < pos_low[i]:
                pos_low[i] = joint.position  

            feedbacks[i][0] = joint.torque 
            feedbacks[i][1] = joint.velocity
            feedbacks[i][2] = joint.position
            #print(feedbacks)

            print(str(joint.ID) + ' |       Pos : '+ str(joint.position) )
            print(str(joint.ID) + ' |  HIGH Pos : '+ str(round(pos_high[i],4)) + ' |  LOW Pos : ' + str(round(pos_low[i],4)))
    if mc is not None :
        mc.send_feedbacks(feedbacks)


def Update(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None) :
    Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
    for _,obj in Named:
        obj.update() 

def set_zero(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None) :
        Update(L_waist,R_waist,L_hip,R_hip,L_knee,R_knee,L_ankle,R_ankle)
        Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
        for _,obj in Named:
                obj.set_zero_position()
                print(str(obj.ID)+ " : set zero")
                time.sleep(2)


#################################################### set mode            
def  impedance_mode(dev,pos,K_val=0,B_val=0) :
    Update(dev)
    dev.set_impedance_gains_real_unit(K=K_val,B=B_val)
    dev.position = pos

def ffb_mode(dev,pos=0,torque=0,K_val=0,B_val=0) :
    Update(dev)
    dev.set_impedance_gains_real_unit_full_state_feedback(K=K_val, B=B_val)
    dev.torque = torque
    dev.position = pos


def current_mode(dev,torque = 0) :
    Update(dev)
    dev.set_current_gains()
    dev.torque = torque

def speed_mode(dev,speed = 0,D_val=0) :
    Update(dev)
    dev.set_speed_gains(kd=D_val)
    dev.velocity = speed


if __name__ == '__main__':

    with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['R_waist']) as R_waist :
     with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['L_waist']) as L_waist :
      with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID=Motor['L_hip']) as L_hip :
       with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['R_hip']) as R_hip :
        with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID=Motor['L_knee']) as L_knee :
         with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['R_knee']) as R_knee :
          with TMotorManager_mit_can(motor_type=Type['ankle'] , motor_ID= Motor['L_ankle']) as L_ankle :
           with TMotorManager_mit_can(motor_type=Type['ankle'] , motor_ID= Motor['R_ankle']) as R_ankle :  
            time.sleep(3)
            set_zero(R_waist,R_hip,R_knee,L_waist,L_hip,L_knee)
            #set_zero(R_waist=R_waist,R_hip=R_hip,R_knee=R_knee,R_ankle=R_ankle,L_waist=L_waist,L_hip=L_hip,L_knee=L_knee,L_ankle=L_ankle)
            loop = SoftRealtimeLoop(dt = 0.001, report= True, fade=0.0)
            
            for t in loop :
            #  read_only(R_waist=R_waist,R_hip=R_hip,R_knee=R_knee,R_ankle=R_ankle,L_waist=L_waist,L_hip=L_hip,L_knee=L_knee,L_ankle=L_ankle)
                if t < 1.0:
                    tor = 0.0 
                    pos = 0.0   
                    pos1 = 0.0   
                else:        
                    tor = 0.0
                    pos = 0.1*np.sin(4*t)
                    pos1 = 0.2*np.sin(4*t)
                ffb_mode(R_waist,pos,tor,22,1.7)   #2
                ffb_mode(R_hip,pos1,tor,22,1)     #3
                ffb_mode(R_knee,pos1,tor,22,1)     #4
                ffb_mode(L_waist,pos,tor,22,1.7)     #6
                ffb_mode(R_hip,pos1,tor,22,1)     #7
                ffb_mode(L_knee,pos1,tor,22,1)        #8
                
                #current_mode(test,tor)
               # print(test.torque)
              #  print('pos :',test.position)
                
    
        

            
