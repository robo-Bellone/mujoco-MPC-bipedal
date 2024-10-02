from motor_control import MotorControl
import time

def get_Smemory(mc = None) :
   if mc is not None :
        data = mc.read_commands()
        command = [{'position': 0, 'velocity': 0, 'torque': 0} for _ in range(8)]
        for i in range(8) :
            if i == 1 or i == 5 :
                command[i]['position'] = -data[i*3+2]
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

if __name__ == "__main__":
    mc = MotorControl("ControlSharedMemory")
    while 1 :
     command = get_Smemory(mc)
     for i in range(8) :
         print(command[Motor['R_waist']-2]['position'])
         print(command[Motor['R_hip']-2]['position'])
         print(command[Motor['R_knee']-2]['position'])
         print(command[Motor['R_ankle']-2]['position'])

    # # 18개 모터의 피드백 데이터 초기화
    # feedbacks = [(20 * i, 200 * i, 2000 * i) for i in range(1,9,1)]

    # # 지속적으로 피드백 데이터 업데이트 및 커맨드 데이터 읽기
    # while True:
    #     # 피드백 보내기
    #     mc.send_feedbacks(feedbacks)
       

    #     # 커맨드 읽기
    #     commands = mc.read_commands()
    #     print("Received Commands:")
    #     for i in range(8):
    #         torque, speed, position = commands[3*i:3*i+3]
    #        # print(f"Motor {i}: Torque={torque}, Speed={speed}, Position={position}")
    #         print(commands)
    #     time.sleep(1)  # 데이터 갱신 주기 설정

