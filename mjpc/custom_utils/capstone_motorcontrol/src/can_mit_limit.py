import can
import time
import csv
import traceback
from collections import namedtuple
from enum import Enum
from math import isfinite
import numpy as np
import warnings
import os



MIT_Params = {
        'ERROR_CODES':{
            0 : 'No Error',
            1 : 'Over temperature fault',
            2 : 'Over current fault',
            3 : 'Over voltage fault',
            4 : 'Under voltage fault',
            5 : 'Encoder fault',
            6 : 'Phase current unbalance fault (The hardware may be damaged)'
        },
       
        'AK60-6':{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -50.0,
            'V_max' : 50.0,
            'T_min' : -15.0,
            'T_max' : 15.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 5.0,
            'Kt_TMotor' : 0.068, # from TMotor website (actually 1/Kvll)
            'Current_Factor' : 0.59, # # UNTESTED CONSTANT!
            'Kt_actual': 0.087, # UNTESTED CONSTANT!
            'GEAR_RATIO': 6.0, 
            'Use_derived_torque_constants': False, # true if you have a better model

        },
        'gim4310' :{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -8.8,
            'V_max' : 8.8,
            'T_min' : -18.0,
            'T_max' : 18.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 10.0,
            'Kt_TMotor' : 1.5, # from TMotor website (actually 1/Kvll)
            'Current_Factor' : 36, # UNTESTED CONSTANT!
            'Kt_actual': 1.5, # UNTESTED CONSTANT!
            'GEAR_RATIO': 36.0, 
            'Use_derived_torque_constants': False, # true if you have a better model

        },

        'Space_max' : [0, 0, 
                       0.3141, 
                       1.4021, 
                       2.4226, 
                       0.87, 
                       0.4061, 
                       0.7753, 
                       3.3026, 
                       0.9474],
                            
        'Space_min' : [0, 0, 
                       -0.3954, 
                       -0.8153, 
                       -3.2862, 
                       -1.0485, 
                       -0.4274, 
                       -1.3548, 
                       -2.4748, 
                       -0.9672]

}
MIT_motor_state = namedtuple('motor_state', 'position velocity current temperature error')
LOG_VARIABLES = [
    "output_angle", 
    "output_velocity", 
    "output_acceleration", 
    "current",
    "output_torque"
]

class motor_state:
    
    def __init__(self,position, velocity, current, temperature, error, acceleration):
        """
        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            current: current in amps
            temperature: temperature in degrees C
            error: error code, 0 means no error
        """
        self.set_state(position, velocity, current, temperature, error, acceleration)

    def set_state(self, position, velocity, current, temperature, error, acceleration):
        """
        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            current: current in amps
            temperature: temperature in degrees C
            error: error code, 0 means no error
        """
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temperature = temperature
        self.error = error
        self.acceleration = acceleration

    def set_state_obj(self, other_motor_state):
        """
        Args:
            other_motor_state: The other motor state object with values to set this motor state object's values to.
        """
        self.position = other_motor_state.position
        self.velocity = other_motor_state.velocity
        self.current = other_motor_state.current
        self.temperature = other_motor_state.temperature
        self.error = other_motor_state.error
        self.acceleration = other_motor_state.acceleration
        
class MIT_command:
    def __init__(self, position, velocity, kp, kd, current):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            kp: Position gain
            kd: Velocity gain
            current: Current in amps
        """
        self.position = position
        self.velocity = velocity
        self.kp = kp
        self.kd = kd
        self.current = current

class motorListener(can.Listener):
    """Python-can listener object, with handler to be called upon reception of a message on the CAN bus"""
    def __init__(self, canman, motor):
        """
        Sets stores can manager and motor object references
        
        Args:
            canman: The CanManager object to get messages from
            motor: The TMotorCANManager object to update
        """
        self.canman = canman
        self.bus = canman.bus
        self.motor = motor

    def on_message_received(self, msg):
        """
        Updates this listener's motor with the info contained in msg, if that message was for this motor.

        args:
            msg: A python-can CAN message
        """
        data = bytes(msg.data)
        ID = data[0]
        if ID == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_MIT_message(data, self.motor.type))
           
   
    
class CAN_Manager(object):
    debug = False
    _instance = None
    
    def __new__(cls):
        """
        Makes a singleton object to manage a socketcan_native CAN bus.
        """
        if not cls._instance:
            cls._instance = super(CAN_Manager, cls).__new__(cls)
            print("Initializing CAN Manager" )
            os.system( 'sudo /sbin/ip link set can0 down' )
            os.system('sudo tc qdisc add dev can0 root fq_codel')
            os.system('sudo ifconfig can0 txqueuelen 50000')
            os.system( 'sudo /sbin/ip link set can0 up type can bitrate 1000000' )
            cls._instance.bus = can.interface.Bus(channel='can0', bustype='socketcan')# bustype='socketcan_native')
            # create a python-can notifier object, which motors can later subscribe to
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            
            cls._instance.bus.flush_tx_buffer()
            print("Connected on: " + str(cls._instance.bus))

            if os.path.exists('can_data.csv'):
                os.remove(f'can_data.csv')
        return cls._instance

    def __init__(self):
        pass
        
    def __del__(self):
        os.system( 'sudo /sbin/ip link set can0 down')


    def save_can_data_to_csv(self, csv_filename, can_message, direction):
        with open(csv_filename, 'a', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            if csvfile.tell() == 0:
                csvwriter.writerow(['Timestamp', 'Direction', 'ID', 'Data'])
            hex_data = [hex(byte) for byte in can_message.data]
            csvwriter.writerow([time.time(), direction, hex(can_message.arbitration_id), hex_data])

         

    # subscribe a motor object to the CAN bus to be updated upon message reception
    def add_motor(self, motor):
        """
        Subscribe a motor object to the CAN bus to be updated upon message reception

        Args:
            motor: The TMotorManager object to be subscribed to the notifier
        """
        self.notifier.add_listener(motorListener(self, motor))

    @staticmethod
    def limit_value(value, min, max):
        if value >= max:
            return max
        elif value <= min:
            return min
        else:
            return value

    # interpolates a floating point number to fill some amount of the max size of unsigned int, 
    # as specified with the num_bits
    @staticmethod
    def float_to_uint(x,x_min,x_max,num_bits):
        span = x_max-x_min
        bitratio = float((1<<num_bits)/span)
        x = CAN_Manager.limit_value(x,x_min,x_max-(2/bitratio))
        # (x - x_min)*(2^num_bits)/span
        
        return CAN_Manager.limit_value(int((x- x_min)*( bitratio )),0,int((x_max-x_min)*bitratio) )
    @staticmethod
    def uint_to_float(x,x_min,x_max,num_bits):
        span = x_max-x_min
        # (x*span/(2^num_bits -1)) + x_min
        return float(x*span/((1<<num_bits)-1) + x_min)

    def send_MIT_message(self, motor_id, data):
        DLC = len(data)
        assert (DLC <= 8), ('Data too long in message for motor ' + str(motor_id))
        
        if self.debug:
            print('ID: ' + str(hex(motor_id)) + '   Data: ' + '[{}]'.format(', '.join(hex(d) for d in data)) )
        
        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
            self.save_can_data_to_csv('can_data.csv', message, 'send')
            if self.debug:

                print("    Message sent on " + str(self.bus.channel_info) )
        except can.CanError:
            if self.debug:
                print("    Message NOT sent")
        
                
      

    def power_on(self, motor_id):
        self.send_MIT_message(motor_id, [ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0XFC])
    def power_off(self, motor_id):
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD])
    def zero(self, motor_id):
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
    
    def power_on_gim(self, motor_id):
        self.send_MIT_message(motor_id,[ 0x91, 0x00, 0x00, 0x00, 0x00 ,0x00, 0x00, 0x00])
    def powr_off_gim(self, motor_id):
        self.send_MIT_message(motor_id, [0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    # basically an impedance controller
    def MIT_controller(self, motor_id, motor_type, position, velocity, Kp, Kd, I):
        """
        Sends an MIT style control signal to the motor. This signal will be used to generate a 
        current for the field-oriented controller on the motor control chip, given by this expression:

            q_control = Kp*(position - current_position) + Kd*(velocity - current_velocity) + I

        Args:
            motor_id: The CAN ID of the motor to send the message to
            motor_type: A string noting the type of motor, ie 'AK80-9'
            position: The desired position in rad
            velocity: The desired velocity in rad/s
            Kp: The position gain
            Kd: The velocity gain
            I: The additional current
        """
        position_uint16 = CAN_Manager.float_to_uint(position, MIT_Params[motor_type]['P_min'], 
                                                    MIT_Params[motor_type]['P_max'], 16)
        velocity_uint12 = CAN_Manager.float_to_uint(velocity, MIT_Params[motor_type]['V_min'], 
                                                    MIT_Params[motor_type]['V_max'], 12)
        Kp_uint12 = CAN_Manager.float_to_uint(Kp, MIT_Params[motor_type]['Kp_min'], 
                                                    MIT_Params[motor_type]['Kp_max'], 12)
        Kd_uint12 = CAN_Manager.float_to_uint(Kd, MIT_Params[motor_type]['Kd_min'], 
                                                    MIT_Params[motor_type]['Kd_max'], 12)
        I_uint12 = CAN_Manager.float_to_uint(I, MIT_Params[motor_type]['T_min'], 
                                                    MIT_Params[motor_type]['T_max'], 12)

        data = [
            position_uint16 >> 8,
            position_uint16 & 0x00FF,
            (velocity_uint12) >> 4,
            ((velocity_uint12&0x00F)<<4) | (Kp_uint12) >> 8,
            (Kp_uint12&0x0FF),
            (Kd_uint12) >> 4,
            ((Kd_uint12&0x00F)<<4) | (I_uint12) >> 8,
            (I_uint12&0x0FF)
        ]
        self.send_MIT_message(motor_id, data)

    # convert data recieved from motor in byte format back into floating point numbers in real units
    def parse_MIT_message(self, data, motor_type):
        """
        Takes a RAW MIT message and formats it into readable floating point numbers.

        Args:
            data: the bytes of data from a python-can message object to be parsed
            motor_type: A string noting the type of motor, ie 'AK80-9'

        Returns:
            An MIT_Motor_State namedtuple that contains floating point values for the 
            position, velocity, current, temperature, and error in rad, rad/s, amps, and *C.
            0 means no error. 
            
            Notably, the current is converted to amps from the reported 
            'torque' value, which is i*Kt. This allows control based on actual q-axis current,
            rather than estimated torque, which doesn't account for friction losses.
        """
        assert len(data) == 8 or len(data) == 6, 'Tried to parse a CAN message that was not Motor State in MIT Mode'
        temp = None
        error = None
        position_uint = data[1] <<8 | data[2]
        velocity_uint = ((data[3] << 8) | (data[4]>>4) <<4 ) >> 4
        current_uint = (data[4]&0x0F)<<8 | data[5]
        
        if len(data)  == 8:
            temp = int(data[6])
            error = int(data[7])

        position = CAN_Manager.uint_to_float(position_uint, MIT_Params[motor_type]['P_min'], 
                                            MIT_Params[motor_type]['P_max'], 16)
        velocity = CAN_Manager.uint_to_float(velocity_uint, MIT_Params[motor_type]['V_min'], 
                                            MIT_Params[motor_type]['V_max'], 12)
        current = CAN_Manager.uint_to_float(current_uint, MIT_Params[motor_type]['T_min'], 
                                            MIT_Params[motor_type]['T_max'], 12)

        if self.debug:
            print('  Position: ' + str(position))
            print('  Velocity: ' + str(velocity))
            print('  Current: ' + str(current))
            if (temp is not None) and (error is not None):
                print('  Temp: ' + str(temp))
                print('  Error: ' + str(error))

        #self.save_can_data_to_csv('can_data.csv', can_message, 'recv')

        return MIT_motor_state(position, velocity, current, temp, error)
    
class _TMotorManState(Enum):
    """
    An Enum to keep track of different control states
    """
    IDLE = 0
    IMPEDANCE = 1
    CURRENT = 2
    FULL_STATE = 3
    SPEED = 4


# the user-facing class that manages the motor.
class TMotorManager_mit_can():

    def __init__(self, motor_type='AK60-6', motor_ID=1, max_mosfett_temp=50, CSV_file=None, log_vars = LOG_VARIABLES):
        """
        Sets up the motor manager. Note the device will not be powered on by this method! You must
        call __enter__, mostly commonly by using a with block, before attempting to control the motor.

        Args:
            motor_type: The type of motor being controlled, ie AK80-9.
            motor_ID: The CAN ID of the motor.
            max_mosfett_temp: temperature of the mosfett above which to throw an error, in Celsius
            CSV_file: A CSV file to output log info to. If None, no log will be recorded.
            log_vars: The variables to log as a python list. The full list of possibilities is
                - "output_angle"
                - "output_velocity"
                - "output_acceleration"
                - "current"
                - "output_torque"
                - "motor_angle"
                - "motor_velocity"
                - "motor_acceleration"
                - "motor_torque"
        """
        self.type = motor_type
        self.ID = motor_ID
        self.csv_file_name = CSV_file
        print("Initializing device: " + self.device_info_string())

        self._motor_state = motor_state(0.0,0.0,0.0,0.0,0.0,0.0)
        self._motor_state_async = motor_state(0.0,0.0,0.0,0.0,0.0,0.0)
        self._command = MIT_command(0.0,0.0,0.0,0.0,0.0)
        self._control_state = _TMotorManState.IDLE
        self._times_past_position_limit = 0
        self._times_past_current_limit = 0
        self._times_past_velocity_limit = 0
        self._angle_threshold_max = MIT_Params["Space_max"][self.ID] - 0.3 # radians, only really matters if the motor's going super fast
        self._angle_threshold_min = MIT_Params["Space_min"][self.ID] + 0.3 
        self._current_threshold = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]['T_max']) - 3.0 # A, only really matters if the current changes quick
        self._velocity_threshold = MIT_Params[self.type]['V_max'] - 2.0 # radians, only really matters if the motor's going super fast
        self._old_pos = None
        self._old_curr = 0.0
        self._old_vel = 0.0
        self._old_current_zone = 0
        self.max_temp = max_mosfett_temp # max temp in deg C, can update later

        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time = None
        self._updated = False
        self.SF = 1.0
        
        self.log_vars = log_vars
        self.LOG_FUNCTIONS = {
            "output_angle" : self.get_output_angle_radians, 
            "output_velocity" : self.get_output_velocity_radians_per_second, 
            "output_acceleration" : self.get_output_acceleration_radians_per_second_squared, 
            "current" : self.get_current_qaxis_amps,
            "output_torque": self.get_output_torque_newton_meters,
            "motor_angle" : self.get_motor_angle_radians, 
            "motor_velocity" : self.get_motor_velocity_radians_per_second, 
            "motor_acceleration" : self.get_motor_acceleration_radians_per_second_squared, 
            "motor_torque": self.get_motor_torque_newton_meters 
        }
        
        self._canman = CAN_Manager()
        self._canman.add_motor(self)      
            
    def __enter__(self):
        print('Turning on control for device: ' + self.device_info_string())
        if self.csv_file_name is not None:
            with open(self.csv_file_name,'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"]+self.log_vars)
            self.csv_file = open(self.csv_file_name,'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)

        self.power_on()
        self._send_command()
        self._entered = True
        if not self.check_can_connection():
            raise RuntimeError("Device not connected: " + str(self.device_info_string()))
        return self

    def __exit__(self, etype, value, tb):
        """
        Used to safely power the motor off and close the log file (if specified).
        """
        print('Turning off control for device: ' + self.device_info_string())
        self.power_off()
        self.power_off()
        self.power_off()
     

        if self.csv_file_name is not None:
            self.csv_file.__exit__(etype, value, tb)

        if not (etype is None):
            traceback.print_exception(etype, value, tb)


    def TMotor_current_to_qaxis_current(self, iTM):
        return MIT_Params[self.type]['Current_Factor']*iTM/(MIT_Params[self.type]['GEAR_RATIO']*MIT_Params[self.type]['Kt_TMotor'])
    
    def qaxis_current_to_TMotor_current(self, iq):
        return iq*(MIT_Params[self.type]['GEAR_RATIO']*MIT_Params[self.type]['Kt_TMotor'])/MIT_Params[self.type]['Current_Factor']

    # this method is called by the handler every time a message is recieved on the bus
    # from this motor, to store the most recent state information for later
    def _update_state_async(self, MIT_state):
        """
        This method is called by the handler every time a message is recieved on the bus
        from this motor, to store the most recent state information for later
        
        Args:
            MIT_state: The MIT_Motor_State namedtuple with the most recent motor state.

        Raises:
            RuntimeError when device sends back an error code that is not 0 (0 meaning no error)
        """
        if MIT_state.error != 0 and MIT_state.error is not None :
            raise RuntimeError('Driver board error for device: ' + self.device_info_string() + ": " + MIT_Params['ERROR_CODES'][MIT_state.error])

        now = time.time()
        dt = self._last_update_time - now
        self._last_update_time = now
        acceleration = (MIT_state.velocity - self._motor_state_async.velocity)/dt

        # The "Current" supplied by the controller is actually current*Kt, which approximates torque.
        self._motor_state_async.set_state(MIT_state.position, MIT_state.velocity, self.TMotor_current_to_qaxis_current(MIT_state.current), MIT_state.temperature, MIT_state.error, acceleration)
        self._updated = True

    
    # this method is called by the user to synchronize the current state used by the controller
    # with the most recent message recieved
    def update(self):
        """
        This method is called by the user to synchronize the current state used by the controller
        with the most recent message recieved, as well as to send the current command.
        """

        # check that the motor is safely turned on
        if not self._entered:
            raise RuntimeError("Tried to update motor state before safely powering on for device: " + self.device_info_string())

     #if self.get_temperature_celsius() > self.max_temp:
     #      raise RuntimeError("Temperature greater than {}C for device: {}".format(self.max_temp, self.device_info_string()))

        # check that the motor data is recent
        # print(self._command_sent)
        now = time.time()
        if (now - self._last_command_time) < 0.25 and ( (now - self._last_update_time) > 0.1):
            # print("State update requested but no data recieved from motor. Delay longer after zeroing, decrease frequency, or check connection.")
            warnings.warn("State update requested but no data from motor. Delay longer after zeroing, decrease frequency, or check connection. " + self.device_info_string(), RuntimeWarning)
        else:
            self._command_sent = False

        # artificially extending the range of the position, current, and velocity that we track
        P_max = MIT_Params["Space_max"][self.ID]- 0.1
        P_min = MIT_Params['Space_min'][self.ID]+ 0.1
        I_max =  self.TMotor_current_to_qaxis_current(MIT_Params[self.type]['T_max']) + 1.0
        V_max =  MIT_Params[self.type]['V_max']+ 0.01
        
        if self._old_pos is None:
            self._old_pos = self._motor_state_async.position
        old_pos = self._old_pos
        old_curr = self._old_curr
        old_vel = self._old_vel
        
        new_pos = self._motor_state_async.position
        new_curr = self._motor_state_async.current
        new_vel = self._motor_state_async.velocity

        thresh_pos_max = self._angle_threshold_max
        thresh_pos_min = self._angle_threshold_min
        thresh_curr = self._current_threshold
        thresh_vel = self._velocity_threshold
        
        curr_command = self._command.current

        actual_current = new_curr

        # The TMotor will wrap around to -max at the limits for all values it returns!! Account for this
        if (thresh_pos_max <= new_pos and new_pos <= P_max) :
            self._motor_state.position = thresh_pos_max
        elif (P_min <= old_pos and old_pos <= thresh_pos_min):
            self._motor_state.position = thresh_pos_min
        elif (thresh_pos_max <= old_pos and old_pos <= P_max) :
            self._motor_state.position = thresh_pos_max
        elif (-P_min <= new_pos and new_pos <= thresh_pos_min) :
            self._motor_state.position = thresh_pos_max
            

        # current is basically the same as position, but if you instantly command a switch it can actually change fast enough
        # to throw this off, so that is accounted for too. We just put a hard limit on the current to solve current jitter problems.
        if (thresh_curr <= new_curr and new_curr <= I_max) and (-I_max <= old_curr and old_curr <= -thresh_curr):
            # self._old_current_zone = -1
            # if (thresh_curr <= curr_command and curr_command <= I_max):
            #     self._times_past_current_limit -= 1
            if curr_command > 0:
                actual_current = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]['T_max'])
            elif curr_command < 0:
                actual_current = -self.TMotor_current_to_qaxis_current(MIT_Params[self.type]['T_max'])
            else:
                actual_current = -self.TMotor_current_to_qaxis_current(MIT_Params[self.type]['T_max'])
            new_curr = actual_current
        elif (thresh_curr <= old_curr and old_curr <= I_max) and (-I_max <= new_curr and new_curr <= -thresh_curr):
            # self._old_current_zone = 1
            # if not (-I_max <= curr_command and curr_command <= -thresh_curr):
            #     self._times_past_current_limit += 1
            if curr_command > 0:
                actual_current = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]['T_max'])
            elif curr_command < 0:
                actual_current = -self.TMotor_current_to_qaxis_current(MIT_Params[self.type]['T_max'])
            else:
                actual_current = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]['T_max'])
            new_curr = actual_current
            

        # velocity should work the same as position
        if (thresh_vel <= new_vel and new_vel <= V_max) and (-V_max <= old_vel and old_vel <= -thresh_vel):
            self._times_past_velocity_limit -= 1
        elif (thresh_vel <= old_vel and old_vel <= V_max) and (-V_max <= new_vel and new_vel <= -thresh_vel) :
            self._times_past_velocity_limit += 1
            
        # update expanded state variables
        self._old_pos = new_pos
        self._old_curr = new_curr
        self._old_vel = new_vel

        self._motor_state.set_state_obj(self._motor_state_async)
        self._motor_state.position += self._times_past_position_limit*2*MIT_Params[self.type]['P_max']
        self._motor_state.current = actual_current
        self._motor_state.velocity += self._times_past_velocity_limit*2*MIT_Params[self.type]['V_max']
        
        # send current motor command
        self._send_command()
        

        # writing to log file
        if self.csv_file_name is not None:
            self.csv_writer.writerow([self._last_update_time - self._start_time] + [self.LOG_FUNCTIONS[var]() for var in self.log_vars])

        self._updated = False

    
    # sends a command to the motor depending on whats controlm mode the motor is in
    def _send_command(self):
        """
        Sends a command to the motor depending on whats controlm mode the motor is in. This method
        is called by update(), and should only be called on its own if you don't want to update the motor state info.

        Notably, the current is converted to amps from the reported 'torque' value, which is i*Kt. 
        This allows control based on actual q-axis current, rather than estimated torque, which 
        doesn't account for friction losses.
        """
        if self._control_state == _TMotorManState.FULL_STATE:
            self._canman.MIT_controller(self.ID,self.type, self._command.position, self._command.velocity, self._command.kp, self._command.kd, self.qaxis_current_to_TMotor_current(self._command.current))
        elif self._control_state == _TMotorManState.IMPEDANCE:
            self._canman.MIT_controller(self.ID,self.type, self._command.position, self._command.velocity, self._command.kp, self._command.kd, 0.0)
        elif self._control_state == _TMotorManState.CURRENT:
            self._canman.MIT_controller(self.ID, self.type, 0.0, 0.0, 0.0, 0.0, self.qaxis_current_to_TMotor_current(self._command.current))
        elif self._control_state == _TMotorManState.IDLE:
            self._canman.MIT_controller(self.ID,self.type, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif self._control_state == _TMotorManState.SPEED:
            self._canman.MIT_controller(self.ID,self.type,0.0,self._command.velocity,0.0,self._command.kd,0.0)
        else:
            raise RuntimeError("UNDEFINED STATE for device " + self.device_info_string())
        self._last_command_time = time.time()

    # Basic Motor Utility Commands
    def power_on(self):
        self._canman.power_on(self.ID)
        self._updated = True
    def power_off(self):
        """Powers off the motor."""
        self._canman.power_off(self.ID)
    def set_zero_position(self):
        self._canman.zero(self.ID)
        self._last_command_time = time.time()


    # getters for motor state
    def get_temperature_celsius(self):
        return self._motor_state.temperature   
    def get_motor_error_code(self):
        return self._motor_state.error
    def get_current_qaxis_amps(self):
        return self._motor_state.current
    def get_output_angle_radians(self):
        return self._motor_state.position
    def get_output_velocity_radians_per_second(self):
        return self._motor_state.velocity
    def get_output_acceleration_radians_per_second_squared(self):
        return self._motor_state.acceleration
    def get_output_torque_newton_meters(self):
        return self.get_current_qaxis_amps()*MIT_Params[self.type]["Kt_actual"]*MIT_Params[self.type]["GEAR_RATIO"]


    # uses plain impedance mode, will send 0.0 for current command.
    def set_impedance_gains_real_unit(self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0):
        """
        Uses plain impedance mode, will send 0.0 for current command in addition to position request.
        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            K: The stiffness in Nm/rad
            B: The damping in Nm/(rad/s)
            ff: A dummy argument for backward compatibility with the dephy library.
        """
        assert(isfinite(K) and MIT_Params[self.type]["Kp_min"] <= K and K <= MIT_Params[self.type]["Kp_max"])
        assert(isfinite(B) and MIT_Params[self.type]["Kd_min"] <= B and B <= MIT_Params[self.type]["Kd_max"])
        self._command.kp = K
        self._command.kd = B
        self._command.velocity = 0.0
        self._control_state = _TMotorManState.IMPEDANCE

    # uses full MIT mode, will send whatever current command is set. 
    def set_impedance_gains_real_unit_full_state_feedback(self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0):
        """"
        Uses full state feedback mode, will send whatever current command is set in addition to position request.
        
        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            K: The stiffness in Nm/rad
            B: The damping in Nm/(rad/s)
            ff: A dummy argument for backward compatibility with the dephy library."""
        assert(isfinite(K) and MIT_Params[self.type]["Kp_min"] <= K and K <= MIT_Params[self.type]["Kp_max"])
        assert(isfinite(B) and MIT_Params[self.type]["Kd_min"] <= B and B <= MIT_Params[self.type]["Kd_max"])
        self._command.kp = K
        self._command.kd = B
        self._control_state = _TMotorManState.FULL_STATE

    def set_current_gains(self, kp=40, ki=400, ff=128, spoof=False):
        """
        Uses plain current mode, will send 0.0 for position gains in addition to requested current.
        
        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            ff: A dummy argument for backward compatibility with the dephy library.
            spoof: A dummy argument for backward compatibility with the dephy library.
        """
        self._control_state = _TMotorManState.CURRENT

    def set_speed_gains(self, kd=1.0):
        """
        Uses plain speed mode, will send 0.0 for position gain and for feed forward current.
        
        Args:
            kd: The gain for the speed controller. Control law will be (v_des - v_actual)*kd = iq
        """
        self._command.kd = kd
        self._control_state = _TMotorManState.SPEED

    def set_output_angle_radians(self, pos):
        """
        Used for either impedance or full state feedback mode to set output angle command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            pos: The desired output position in rads
        """
        # position commands must be within a certain range :/
        # CANNOT Control using impedance mode for angles greater than 12.5 rad!!
        if pos >= MIT_Params["Space_max"][self.ID]:
            pos = MIT_Params["Space_max"][self.ID] - 0.1
        if pos <= MIT_Params["Space_min"][self.ID] :
             pos = MIT_Params["Space_min"][self.ID] + 0.1
        if self._control_state not in [_TMotorManState.IMPEDANCE, _TMotorManState.FULL_STATE]:
            raise RuntimeError("Attempted to send position command without gains for device " + self.device_info_string()) 
        
        self._command.position = pos
    
    def set_output_velocity_radians_per_second(self, vel):
        """
        Used for either speed or full state feedback mode to set output velocity command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(vel) >= MIT_Params[self.type]["V_max"]:
            raise RuntimeError("Cannot control using speed mode for angles with magnitude greater than " + str(MIT_Params[self.type]["V_max"]) + "rad/s!")

        if self._control_state not in [_TMotorManState.SPEED, _TMotorManState.FULL_STATE]:
            raise RuntimeError("Attempted to send speed command without gains for device " + self.device_info_string()) 
        self._command.velocity = vel
    
    def set_output_torque_newton_meters(self, torque):
        """
        Used for either current or MIT Mode to set current, based on desired torque.
        If a more complicated torque model is available for the motor, that will be used.
        Otherwise it will just use the motor's torque constant.
        
        Args:
            torque: The desired output torque in Nm.
        """
        if np.abs(torque) >= MIT_Params[self.type]["T_max"]:
            raise RuntimeError("Cannot control using speed mode for angles with magnitude greater than " + str(MIT_Params[self.type]["T_max"]) + "Nm!")

        self.set_motor_current_qaxis_amps((torque/MIT_Params[self.type]["Kt_actual"]/MIT_Params[self.type]["GEAR_RATIO"]) )
   
    def set_motor_current_qaxis_amps(self, current):
        """
        Used for either current or full state feedback mode to set current command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.
        
        Args:
            current: the desired current in amps.
        """
        if self._control_state not in [_TMotorManState.CURRENT, _TMotorManState.FULL_STATE]:
            raise RuntimeError("Attempted to send current command before entering current mode for device " + self.device_info_string()) 
        self._command.current = current


    # motor-side functions to account for the gear ratio
    def set_motor_torque_newton_meters(self, torque):
        """
        Version of set_output_torque that accounts for gear ratio to control motor-side torque
        
        Args:
            torque: The desired motor-side torque in Nm.
        """
        self.set_output_torque_newton_meters(torque*MIT_Params[self.type]["Kt_actual"])
    def set_motor_angle_radians(self, pos):
        """
        Wrapper for set_output_angle that accounts for gear ratio to control motor-side angle
        
        Args:
            pos: The desired motor-side position in rad.
        """
        self.set_output_angle_radians(pos/(MIT_Params[self.type]["GEAR_RATIO"]) )
    def set_motor_velocity_radians_per_second(self, vel):
        """
        Wrapper for set_output_velocity that accounts for gear ratio to control motor-side velocity
        
        Args:
            vel: The desired motor-side velocity in rad/s.
        """
        self.set_output_velocity_radians_per_second(vel/(MIT_Params[self.type]["GEAR_RATIO"]) )

    def get_motor_angle_radians(self):
        """
        Wrapper for get_output_angle that accounts for gear ratio to get motor-side angle
        
        Returns:
            The most recently updated motor-side angle in rad.
        """
        return self._motor_state.position*MIT_Params[self.type]["GEAR_RATIO"]
    def get_motor_velocity_radians_per_second(self):
        """
        Wrapper for get_output_velocity that accounts for gear ratio to get motor-side velocity
        
        Returns:
            The most recently updated motor-side velocity in rad/s.
        """
        return self._motor_state.velocity*MIT_Params[self.type]["GEAR_RATIO"]
    def get_motor_acceleration_radians_per_second_squared(self):
        """
        Wrapper for get_output_acceleration that accounts for gear ratio to get motor-side acceleration
        
        Returns:
            The most recently updated motor-side acceleration in rad/s/s.
        """
        return self._motor_state.acceleration*MIT_Params[self.type]["GEAR_RATIO"]
    def get_motor_torque_newton_meters(self):
        """
        Wrapper for get_output_torque that accounts for gear ratio to get motor-side torque
        
        Returns:
            The most recently updated motor-side torque in Nm.
        """
        return self.get_output_torque_newton_meters()*MIT_Params[self.type]["GEAR_RATIO"]

    # Pretty stuff
    def __str__(self):
        position_str = '{: 1f}'.format(round(self.position,3)) if self.position is not None else "N/A"
        velocity_str = '{: 1f}'.format(round(self.velocity,3)) if self.velocity is not None else "N/A"
        current_str = '{: 1f}'.format(round(self.current_qaxis,3)) if self.current_qaxis is not None else "N/A"
        torque_str = '{: 1f}'.format(round(self.torque,3)) if self.torque is not None else "N/A"
        temperature_str = '{: 1f}'.format(round(self.temperature,3)) if self.temperature is not None else "N/A"

        return self.device_info_string() + " | Position: " + position_str + " rad | Velocity: " + velocity_str + " rad/s | current: " + current_str + " A | torque: " + torque_str + " Nm | temperature" + temperature_str

    def device_info_string(self):
        """Prints the motor's ID and device type."""
        return str(self.type) + "  ID: " + str(self.ID)

    # Checks the motor connection by sending a 10 commands and making sure the motor responds.
    def check_can_connection(self):
        """
        Checks the motor's connection by attempting to send 10 startup messages.
        If it gets 10 replies, then the connection is confirmed.

        Returns:
            True if a connection is established and False otherwise.
        """
        if not self._entered:
            raise RuntimeError("Tried to check_can_connection before entering motor control! Enter control using the __enter__ method, or instantiating the TMotorManager in a with block.")
        Listener = can.BufferedReader()
        self._canman.notifier.add_listener(Listener)
        for i in range(10):
            self.power_on()
            time.sleep(0.001)
        success = True
        time.sleep(0.1)
        for i in range(10):
            if Listener.get_message(timeout=0.1) is None:
                success = False
        self._canman.notifier.remove_listener(Listener)
        return success



    # controller variables
    temperature = property(get_temperature_celsius, doc="temperature_degrees_C")
    """Temperature in Degrees Celsius"""

    error = property(get_motor_error_code, doc="temperature_degrees_C")
    """Motor error code. 0 means no error."""


    # electrical variables
    current_qaxis = property(get_current_qaxis_amps, set_motor_current_qaxis_amps, doc="current_qaxis_amps_current_only")
    """Q-axis current in amps"""

    # output-side variables
    position = property(get_output_angle_radians, set_output_angle_radians, doc="output_angle_radians_impedance_only")
    """Output angle in rad"""

    velocity = property (get_output_velocity_radians_per_second, set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    """Output velocity in rad/s"""

    acceleration = property(get_output_acceleration_radians_per_second_squared, doc="output_acceleration_radians_per_second_squared")
    """Output acceleration in rad/s/s"""

    torque = property(get_output_torque_newton_meters, set_output_torque_newton_meters, doc="output_torque_newton_meters")
    """Output torque in Nm"""


    # motor-side variables
    position_motorside = property(get_motor_angle_radians, set_motor_angle_radians, doc="motor_angle_radians_impedance_only")
    """Motor-side angle in rad"""
    
    velocity_motorside = property (get_motor_velocity_radians_per_second, set_motor_velocity_radians_per_second, doc="motor_velocity_radians_per_second")
    """Motor-side velocity in rad/s"""

    acceleration_motorside = property(get_motor_acceleration_radians_per_second_squared, doc="motor_acceleration_radians_per_second_squared")
    """Motor-side acceleration in rad/s/s"""

    torque_motorside = property(get_motor_torque_newton_meters, set_motor_torque_newton_meters, doc="motor_torque_newton_meters")
    """Motor-side torque in Nm"""



