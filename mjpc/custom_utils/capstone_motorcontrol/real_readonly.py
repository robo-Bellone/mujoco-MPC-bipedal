from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from src.can_mit_limit import TMotorManager_mit_can
from src.motor_control import MotorControl

import time
import numpy as np
import can
import os

Type = {
    'others':'AK60-6',
    'ankle':'gim4310'
    }

Motor = {
    'R_waist' : 2, 
    'L_waist' : 6,
    'R_hip' : 3, 
    'L_hip' : 7, 
    'R_knee' : 4,  
    'L_knee' : 8,
    'R_ankle' : 5,
    'L_ankle':9
}


##########################      basic            ###########################################################################################################################################
def read_only(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None):
    pos_high = [0,0,0,0,0,0,0,0]
    pos_low = [0,0,0,0,0,0,0,0]

    Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
    Update(L_waist,R_waist,L_hip,R_hip,L_knee,R_knee,L_ankle,R_ankle)
    for i, joint in Named:
        if joint.position > pos_high[i]:
            pos_high[i] = joint.position
        elif joint.position < pos_low[i]:
            pos_low[i] = joint.position  

        
        print(str(joint.ID) + ' |       Pos : '+ str(joint.position) )
        print(str(joint.ID) + ' |  HIGH Pos : '+ str(round(pos_high[i],4)) + ' |  LOW Pos : ' + str(round(pos_low[i],4)))

def Update(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None) :
    Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
    for _,obj in Named:
        obj.update() 

def set_zero(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None) :
    Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
    for _,obj in Named:
        obj.set_zero_position()
        print(str(obj.ID)+ " : set zero")
        time.sleep(2)

def motor_off(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None) :
    Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
    for _,obj in Named:
        obj.power_off()


##########################     setting            ######################################################################################################################################
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

##########################     memory            ######################################################################################################################################
def set_Smemory(mc = None,L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None ) :
    if mc is not None :
        feedbacks = [[0 for _ in range(3)] for _ in range(8)]
        Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
        for i, joint in Named:
            if joint.ID == 3 or joint.ID == 4 or joint.ID == 9  :
                feedbacks[joint.ID - 2][0] = -joint.acceleration 
                feedbacks[joint.ID - 2][1] = -joint.velocity
                feedbacks[joint.ID - 2][2] = -joint.position
            else : 
                feedbacks[joint.ID - 2][0] = joint.acceleration
                feedbacks[joint.ID - 2][1] = joint.velocity
                feedbacks[joint.ID - 2][2] = joint.position


        feedbacks[2][0] = feedbacks[2][0] + feedbacks[1][0] 
        feedbacks[2][1] = feedbacks[2][1] + feedbacks[1][1] 
        feedbacks[2][2] = feedbacks[2][2] + feedbacks[1][2] 

        feedbacks[6][0] = feedbacks[6][0] + feedbacks[5][0] 
        feedbacks[6][1] = feedbacks[6][1] + feedbacks[5][1] 
        feedbacks[6][2] = feedbacks[6][2] + feedbacks[5][2] 
        
        print('Wai : ',feedbacks[0][2],'    ',R_waist.position,'    ',feedbacks[0][1],'    ',R_waist.velocity)
        print('nee : ',feedbacks[2][2],'    ',R_knee.position,'    ',feedbacks[2][1],'    ',R_knee.velocity)
        print('hip : ',feedbacks[1][2],'    ',R_hip.position,'    ',feedbacks[1][1],'    ',R_hip.velocity)
        print('ank : ',feedbacks[3][2],'    ',R_ankle.position,'    ',feedbacks[3][1],'    ',R_ankle.velocity)
        print('')
        print('Wai : ',feedbacks[4][2],'    ',L_waist.position,'    ',feedbacks[4][1],'    ',L_waist.velocity)
        print('nee : ',feedbacks[6][2],'    ',L_knee.position,'    ',feedbacks[6][1],'    ',L_knee.velocity)
        print('hip : ',feedbacks[5][2],'    ',L_hip.position,'    ',feedbacks[5][1],'    ',L_hip.velocity)
        print('ank : ',feedbacks[7][2],'    ',L_ankle.position,'    ',feedbacks[7][1],'    ',L_ankle.velocity)


        mc.send_feedbacks(feedbacks)
        

        

def get_Smemory(mc = None) :
   if mc is not None :
        data = mc.read_commands()
        command = [{'position': 0, 'velocity': 0, 'torque': 0} for _ in range(8)]
        for i in range(8) :
            if i == 1 or i == 2 or i == 7 :            
                command[i]['position'] = -data[i*3+2]
                command[i]['velocity'] = -data[i*3+1]
                command[i]['torque'] = -data[i*3]
               
            else : 
                command[i]['position'] = data[i*3+2]
                command[i]['velocity'] = data[i*3+1]
                command[i]['torque'] = data[i*3]

        command[2]['position'] = command[2]['position'] - command[1]['position']        
        command[2]['velocity'] = command[2]['velocity'] - command[1]['velocity']
        command[2]['torque'] = command[2]['torque'] - command[1]['torque']
        
        command[6]['position'] = command[6]['position'] - command[5]['position']        
        command[6]['velocity'] = command[6]['velocity'] - command[5]['velocity']
        command[6]['torque'] = command[6]['torque'] - command[5]['torque']
        
        return command

if __name__ == '__main__':
  
     with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['R_waist'] , CSV_file='R_waist.csv') as R_waist :
      with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['L_waist'], CSV_file='L_waist.csv') as L_waist :
       with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID=Motor['L_hip'], CSV_file='L_hip.csv') as L_hip :
        with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['R_hip'], CSV_file='R_hip.csv') as R_hip :
         with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID=Motor['L_knee'], CSV_file='L_knee.csv') as L_knee :
          with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['R_knee'], CSV_file='R_knee.csv') as R_knee :
           with TMotorManager_mit_can(motor_type=Type['ankle'] , motor_ID= Motor['L_ankle'], CSV_file='L_ankle.csv') as L_ankle :
            with TMotorManager_mit_can(motor_type=Type['ankle'] , motor_ID= Motor['R_ankle'],CSV_file='R_ankle.csv') as R_ankle :         
                    
                mc = MotorControl("ControlSharedMemory")
                try:
                    time.sleep(1)  
                    loop = SoftRealtimeLoop(dt = 0.001, report= True, fade=0.0)
                    set_zero(R_waist=R_waist,R_hip=R_hip,R_knee=R_knee,R_ankle=R_ankle,L_waist=L_waist,L_hip=L_hip,L_knee=L_knee,L_ankle=L_ankle)
                    for t in loop :
                        command = get_Smemory(mc)
                        Update(R_waist=R_waist,R_hip=R_hip,R_knee=R_knee,R_ankle=R_ankle,L_waist=L_waist,L_hip=L_hip,L_knee=L_knee,L_ankle=L_ankle)
                        #read_only(R_waist=R_waist,R_hip=R_hip,R_knee=R_knee,R_ankle=R_ankle,L_waist=L_waist,L_hip=L_hip,L_knee=L_knee,L_ankle=L_ankle)
                        # #Update(R_waist=R_waist,R_hip=R_hip,R_knee=R_knee,R_ankle=R_ankle,L_waist=L_waist,L_hip=L_hip,L_knee=L_knee,L_ankle=L_ankle)
                        # impedance_mode(L_ankle,command[Motor['L_ankle']-2]['position'],30,5)
                        # impedance_mode(R_waist,command[Motor['R_waist']-2]['position'],10,1)
                        # impedance_mode(L_waist,command[Motor['L_waist']-2]['position'],10,1)
                        # impedance_mode(R_hip,command[Motor['R_hip']-2]['position'],10,1)
                        # impedance_mode(L_hip,command[Motor['L_hip']-2]['position'],10,1)
                        # impedance_mode(R_knee,command[Motor['R_knee']-2]['position'],10,1)
                        # impedance_mode(L_knee,command[Motor['L_knee']-2]['position'],10,1)
                        # impedance_mode(R_ankle,command[Motor['R_ankle']-2]['position'],30,5)
                        

                        # print('Wai : ',command[Motor['R_waist']-2]['position'],'    ',R_waist.position,'    ',R_waist.velocity)
                        # print('nee : ',command[Motor['R_knee']-2]['position'],'    ',R_knee.position,'    ',R_knee.velocity)
                        # print('hip : ',command[Motor['R_hip']-2]['position'],'    ',R_hip.position,'    ',R_hip.velocity)
                        # print('ank : ',command[Motor['R_ankle']-2]['position'],'    ',R_ankle.position,'    ',R_ankle.position)
                        # print('')
                        # print('Wai : ',command[Motor['L_waist']-2]['position'],'    ',L_waist.position,'    ',L_waist.velocity)
                        # print('nee : ',command[Motor['L_knee']-2]['position'],'    ',L_knee.position,'    ',L_knee.velocity)
                        # print('hip : ',command[Motor['L_hip']-2]['position'],'    ',L_hip.position,'    ',L_hip.velocity)
                        # print('ank : ',command[Motor['L_ankle']-2]['position'],'    ',L_ankle.position,'    ',L_ankle.velocity)

                        set_Smemory(mc=mc,R_waist=R_waist,R_hip=R_hip,R_knee=R_knee,R_ankle=R_ankle,L_waist=L_waist,L_hip=L_hip,L_knee=L_knee,L_ankle=L_ankle)
                
                finally:
                    for i in range(5) :
                        R_waist.power_off()
                        R_hip.power_off()
                        R_knee.power_off()
                        R_ankle.power_off()

                        L_waist.power_off()
                        L_hip.power_off()
                        L_knee.power_off()
                        L_ankle.power_off()
                        time.sleep(0.01)
                        os.system( 'sudo /sbin/ip link set can0 down')

                
