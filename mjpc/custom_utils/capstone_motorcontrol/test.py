from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp
from src.can_mit import TMotorManager_mit_can
from src.can_mit import CAN_Manager 
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
    'L_hip' : 7, 
    'R_knee' : 4,  
    'L_knee' : 8,
    'R_ankle' : 5,
    'L_ankle':9
}

# waist : 0.255 ~ -0.5232
# hip : 1.438 ~ -0.556 => 1.4 ~ -0.5
# knee : 1.95 ~3.387

def position_tracking(dev,dev2):
    #dev.set_impedance_gains_real_unit(K=5,B=0.5)
    dev2.set_impedance_gains_real_unit(K=5,B=0.5)
 #   dev3.set_impedance_gains_real_unit(K=1.5,B=0.5)

    print("Starting position tracking demo. Press ctrl+C to quit.")
    loop = SoftRealtimeLoop(dt = 0.01, report= True, fade=0.0)
    for t in loop :
      #  dev.update()
        dev2.update()
      #  dev3.update()

        if t < 1.0:
       #     dev.position = 0.1
            dev2.position = 0.0
        #    dev3.position = 0.1

        else: 
            radian = 0.4*np.sin(np.pi*t)
            print(radian)

         #   dev.positon = 0.1
            dev2.position =  radian
         #   dev3.position = 0.1

        print( '  ' + str(dev2.position) + '  ')
    del loop


def read_only(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None):
    pos_high = [0,0,0,0,0,0,0,0]
    pos_low = [0,0,0,0,0,0,0,0]
    Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]

    loop = SoftRealtimeLoop(dt = 0.001, report= True, fade=0.0)
    for t in loop : 
        Update(L_waist,R_waist,L_hip,R_hip,L_knee,R_knee,L_ankle,R_ankle)
        for i, joint in Named:
            print(round(joint.position,4))
            if joint.position > pos_high[i]:
               pos_high[i] = joint.position
            elif joint.position < pos_low[i]:
                pos_low[i] = joint.position  

            
            print(str(joint.ID) + ' |  HIGH Pos : '+ str(round(pos_high[i],4)) + ' |  LOW Pos : ' + str(round(pos_low[i],4)))
    del loop         


def Update(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None) :
    Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
    for _,obj in Named:
        obj.update() 

def set_zero(L_waist=None,R_waist=None,L_hip=None,R_hip=None,L_knee=None,R_knee=None,L_ankle=None,R_ankle=None) :
    Named = [(index, name) for index, name in enumerate([L_waist, R_waist, L_hip, R_hip, L_knee, R_knee, L_ankle, R_ankle]) if name is not None]
    while 1 :
        for _,obj in Named:
            obj.set_zero_position()
            print(str(obj.ID)+ " : set zero")
            time.sleep(2)
            
           


 #   L_waist.update()
 #   R_waist.update()
 #   L_hip.update()
 #   R_hip.update()
 #   L_knee.update()
 #   R_knee.update()
 #   L_ankle.update()
 #   R_ankle.update()       

if __name__ == '__main__':
    time.sleep(2)
    with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['R_waist']) as R_waist :
#    with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['L_waist']) as R_waist :
#     with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['L_hip']) as L_hip :
     with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['R_hip']) as R_hip :
#       with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['L_knee']) as L_knee :
    #    with TMotorManager_mit_can(motor_type=Type['others'] , motor_ID= Motor['R_knee']) as R_knee :
    #   with TMotorManager_mit_can(motor_type=Type['ankle'] , motor_ID= Motor['L_ankle']) as L_ankle :
    #     with TMotorManager_mit_can(motor_type=Type['ankle'] , motor_ID= Motor['R_ankle']) as R_ankle :
          position_tracking(R_waist,R_hip)
     #   set_zero(R_hip)        
                   

            
