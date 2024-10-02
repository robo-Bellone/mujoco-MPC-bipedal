from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from src.can_mit_limit import TMotorManager_mit_can
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
    'L_waist' : 6,
    'R_hip' : 3, 
    'L_hip' : 8, 
    'R_knee' : 4,  
    'L_knee' : 7,
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
            if i == 1 or i == 5 :
                feedbacks[joint.ID - 2][0] = -joint.torque 
                feedbacks[joint.ID - 2][1] = -joint.velocity
                feedbacks[joint.ID - 2][2] = -joint.position
            else : 
                feedbacks[joint.ID - 2][0] = joint.torque 
                feedbacks[joint.ID - 2][1] = joint.velocity
                feedbacks[joint.ID - 2][2] = joint.position
        
        feedbacks[2][0] -= feedbacks[1][0] 
        feedbacks[2][1] -= feedbacks[1][0]
        feedbacks[2][2] -= feedbacks[1][0]

        feedbacks[6][0] -= feedbacks[5][0] 
        feedbacks[6][1] -= feedbacks[5][0]
        feedbacks[6][2] -= feedbacks[5][0]

        mc.send_feedbacks(feedbacks)

def get_Smemory(mc = None) :
   if mc is not None :
        data = mc.read_commands()
        command = [{'position': 0, 'velocity': 0, 'torque': 0} for _ in range(8)]
        for i in range(8) :
            if i == 1 or i == 5 :
                command[i]['position'] = -data[1*3+2]
                command[i]['velocity'] = -data[i*3+1]
                command[i]['torque'] = -data[i*3]
            elif i == 2 or i == 6 :
                command[i]['position'] = data[i*3+2] + command[i]['position']
                command[i]['velocity'] = data[i*3+1] + command[i]['velocity'] 
                command[i]['torque'] = data[i*3] + command[i]['torque']
               
            else : 
                command[i]['position'] = data[i*3+2]
                command[i]['velocity'] = data[i*3+1]
                command[i]['torque'] = data[i*3]
        return command

if __name__ == '__main__':

           with TMotorManager_mit_can(motor_type=Type['ankle'] , motor_ID=5) as R_knee :
            time.sleep(3)          
            mc = MotorControl("ControlSharedMemory")
            loop = SoftRealtimeLoop(dt = 0.001, report= True, fade=0.0)
            for t in loop :
                if t < 1 :
                    tor =0
                    pos =0
                    ffb_mode(R_knee,pos,tor,130,5)
                else : 
                    command = get_Smemory(mc)
                   # ffb_mode(R_knee,0.5,tor,10,5)
                    ffb_mode(R_knee,command[Motor['R_knee']-2]['position'],command[Motor['R_knee']-2]['torque'],130,5)
                    print(command[Motor['R_knee']-2]['position'],'    ',R_knee.position)

            del loop         
        
        

            
