import numpy as np
from enum import Enum
from abc import ABC, abstractmethod

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import time
import RPi.GPIO as GPIO

# multiprocessing
import multiprocessing
GPIO.setwarnings(False)     # disable warnings

# GPIO setup
GPIO.setmode(GPIO.BOARD)
ir_led_pin = 8		
GPIO.setup(ir_led_pin,GPIO.OUT)

# pwm signal for ir led at ir_led_pin
pi_pwm = GPIO.PWM (ir_led_pin, 38000)
pi_pwm.start(0)
pi_pwm.ChangeDutyCycle(50)  # pwm signal duty cycle

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0

# ADC converter object (converts analog voltage to digital signal) from IR-sensor
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))


class SensorTrigState(Enum):
    NO_TRIG = 0
    TRIG = 1
    UNKNOWN = 2


class SensorSample:
    def __init__(self):
        self.value: int = 0
        self.timestamp: int = 0
        self.trig_state = SensorTrigState.NO_TRIG
        
    def set(self, value, timestamp, trig_state):
        self.value = value
        self.timestamp = timestamp
        self.trig_state = trig_state
    
    def get(self):
        return self.value, self.timestamp, self.trig_state


class IrSensor:
    def __init__(self, mcp_channel :int, sensor_trig_threshold: int):
        self.mcp_channel = mcp_channel
        self.sensor_trig_threshold = sensor_trig_threshold
        
    def get_sensor_data(self):
        # read sensor value and timestamp
        value = mcp.read_adc(self.mcp_channel)
        timestamp = round(time.time()*1000)
        
        # evalute readout value to determine sensor trig
        trig_state = SensorTrigState.NO_TRIG
        if(value < self.sensor_trig_threshold):   # detect sensor trig. below threshold == trig, above threshold = no trig
            trig_state = SensorTrigState.TRIG    # trig detected
        else:
            trig_state = SensorTrigState.NO_TRIG  # no trig detected
        return value, timestamp, trig_state


class SensorHandler:
    def __init__(self, number_of_sensors: int, max_samples: int):
        self.number_of_sensors = number_of_sensors
        self.max_samples = max_samples
        #self.sensor_logs = np.array([[SensorSample()]*max_samples] * self.number_of_sensors)    # create number of objects needed to store the buffered sensor data
        self.sensor_logs = np.array([[SensorSample() for _ in range(max_samples)] for _ in range(self.number_of_sensors)])    # create number of objects needed to store the buffered sensor data
        
    def register_sample(self, sensor_id, index, value, timestamp, trig_state):
        self.sensor_logs[sensor_id][index].set(value, timestamp, trig_state)    
    
    def get_sample(self, sensor_id, index):
        return self.sensor_logs[sensor_id][index]
    
    def get_sensor_logs(self):
        return self.sensor_logs


class SensorState(ABC):
    # store timestamp when state was reached
    last_updated: int   # class variable  which all instaces of the class can access (same variable)
    
    @abstractmethod
    def change_state(self):
        pass
    
    @abstractmethod
    def reset_state(self):
        pass
    

class MotionDetectionState(Enum):
    INIT = 0
    IDLE = 1
    
    EXIT_A = 2
    EXIT_B = 3
    EXIT_C = 4
    EXIT_COMPLETE = 5
    
    ENTRY_A = 6
    ENTRY_B = 7
    ENTRY_C = 8
    ENTRY_COMPLETE = 9


# class Idle(SensorState):
#     def __init__(self, timestamp: int):
#         self.last_updated = timestamp
    
#     def change_state(self, sensor0_trig_state: SensorTrigState, sensor1_trig_state: SensorTrigState, timestamp: int, trig_states_are_same: bool):
#         print("in Idle")
    
#     def reset_state(self):
#         pass



class SensorStateManager:
    current_state: MotionDetectionState = MotionDetectionState.INIT
    
    def __init__(self, number_of_sensors: int, sensor_handler: SensorHandler, num_consecutive_trigs: int, num_consecutive_false_trigs: int):
        # self.number_of_sensors = number_of_sensors
        # self.sensor_handler = sensor_handler
        # self.num_consecutive_trigs = num_consecutive_trigs
        # #self.num_consecutive_false_trigs = num_consecutive_false_trigs
        # self.sensor0_trig_states = []
        # self.sensor1_trig_states = []
        # self.change_state_enabled = False   # enabler to only start evaluate sensor trig states after a specified number of sensor readouts has been done
        # self.init_idle_state_enabled = True
        self.number_of_sensors = number_of_sensors
        self.sensor_handler = sensor_handler
        self.num_consecutive_trigs = num_consecutive_trigs
        #self.num_consecutive_false_trigs = num_consecutive_false_trigs
        self.sensor_trig_states = np.array([[SensorTrigState.NO_TRIG for _ in range(num_consecutive_trigs)] for _ in range(self.number_of_sensors)])    # create number of items needed a buffered number of sensor trig states for evaluation
        self.change_state_is_allowed = False    # enabler to start evaluate sensor trig states only after a specified number of sensor readouts

    def import_sensor_trig_states(self, current_readout_index):
        for i in range(self.num_consecutive_trigs):   # use current sensor readout index get latest sensor trig state as well as a number of previous readouts specified by num_consecutive_trigs variable
            for sensor_id in range(self.number_of_sensors):
                self.sensor_trig_states[sensor_id][i] = self.sensor_handler.get_sample(sensor_id, current_readout_index).trig_state.name
            print(self.sensor_trig_states)
    
    def verify_sensor_trig_continuity(self):
        verified_trig_states = []
        for i in range(self.number_of_sensors):
            if self.sensor_trig_states[i][0] == self.sensor_trig_states[i].all():
                verified_trig_states.append(self.sensor_trig_states[i][0])
            else:
                verified_trig_states.append(SensorTrigState.UNKNOWN.name)
        return verified_trig_states
                
    def detect_motion_direction(self, sensor_states):
        pass
    
    def evaluate_sensor_trig_states(self, current_readout_index: int):
        #consecutive_trig_states_are_same = False   
        
        
        if current_readout_index >= self.num_consecutive_trigs: # enable state change only after a specified number of sensor readouts
            self.change_state_is_allowed = True
        
        if self.change_state_is_allowed:    # enough number of sensor sample readouts performed to be able evalute sensor trig states
            self.import_sensor_trig_states(current_readout_index)
            verified_states = self.verify_sensor_trig_continuity()
            print(verified_states)
            #if(self.current_state == MotionDetectionState.INIT):
                
            
        
                
        
        if self.current_state == MotionDetectionState.INIT:
            pass
            
        
        
        
        
        
        
        # if self.change_state_enabled:  # evaluate trig states for each sensor when state change is allowed
        #     for current_index in range(current_index, current_index - self.num_consecutive_trigs, -1):   # check a number of latest consecutive sensor trig status readouts (number specified by num_consecutive_trigs) individually, for each sensor have the same trig status
        #         for j in range(self.number_of_sensors):
        #             #print("[{:d}][{:d}[{:s}]".format(j, current_index, self.sensor_handler.get_sample(j, current_index).trig_state.name))
        #             if(j == 0): # sensor0
        #                 self.sensor0_trig_states.insert(0, self.sensor_handler.get_sample(j, current_index).trig_state.name)    # add sensor0's trig status to list
        #                 print("sensor0_trig_states:", self.sensor0_trig_states)
        #                 if(len(self.sensor0_trig_states) >= self.num_consecutive_trigs):  # keep FIFO list with size of trigs required to change state (self.num_consecutive_trigs)
        #                     self.sensor0_trig_states.pop()
        #             elif(j == 1):   # sensor1
        #                 self.sensor1_trig_states.insert(0, self.sensor_handler.get_sample(j, current_index).trig_state.name)    # add sensor1's trig status to list
        #                 print("sensor1_trig_states:", self.sensor1_trig_states)
        #                 if(len(self.sensor1_trig_states) >= self.num_consecutive_trigs):  # keep FIFO list with size of trigs required to change state (self.num_consecutive_trigs)
        #                     self.sensor1_trig_states.pop()
        
        #     # evaluate if last number of trig states stored in a separate list for each sensor have the same values  
        #     if all(x == self.sensor0_trig_states[0] for x in self.sensor0_trig_states) and all(y == self.sensor1_trig_states[0] for y in self.sensor1_trig_states):
        #         print("ready to change state")
        #         trig_states_are_same = True
        #         if self.init_idle_state_enabled:    # check if program runs first time when no state has been assigned 
        #             if self.sensor0_trig_states[0] == SensorTrigState.NO_TRIG and self.sensor1_trig_states[0] == SensorTrigState.NO_TRIG:   # check if both sensors are in NO_TRIG state, to prevent changing to Idle if any of the sensors are blocked (in TRIG state) 
        #                 self.init_idle_state_enabled = False    # initialization phase passed
        #                 self.current_state.change_state(self.sensor0_trig_states[0], self.sensor1_trig_states[0], self.sensor_handler.get_sample(0, current_index).timestamp, trig_states_are_same)
        #         else:
        #             self.current_state.change_state(self.sensor0_trig_states[0], self.sensor1_trig_states[0], self.sensor_handler.get_sample(0, current_index).timestamp, trig_states_are_same)
        #     else:
        #         print("NOT ready to change state")
        #         trig_states_are_same = False
        #         self.current_state.change_state(self.sensor0_trig_states[0], self.sensor1_trig_states[0], self.sensor_handler.get_sample(0, current_index).timestamp, trig_states_are_same)
            
    
    
    
     
    # def evaluate_sensor_trig_states(self, current_index: int):
    #     trig_states_are_same = False    # boolean returned to state as a validator if current trig state can be trusted (same trig state for a consecutive number of times) which the state in its turn uses to act which state to transfer to 
    #     if(current_index >= self.num_consecutive_trigs):  # enable state change after a specified number of sensor readouts
    #         self.change_state_enabled = True
        
    #     if self.change_state_enabled:  # evaluate trig states for each sensor when state change is allowed
    #         for current_index in range(current_index, current_index - self.num_consecutive_trigs, -1):   # check a number of latest consecutive sensor trig status readouts (number specified by num_consecutive_trigs) individually, for each sensor have the same trig status
    #             for j in range(self.number_of_sensors):
    #                 #print("[{:d}][{:d}[{:s}]".format(j, current_index, self.sensor_handler.get_sample(j, current_index).trig_state.name))
    #                 if(j == 0): # sensor0
    #                     self.sensor0_trig_states.insert(0, self.sensor_handler.get_sample(j, current_index).trig_state.name)    # add sensor0's trig status to list
    #                     print("sensor0_trig_states:", self.sensor0_trig_states)
    #                     if(len(self.sensor0_trig_states) >= self.num_consecutive_trigs):  # keep FIFO list with size of trigs required to change state (self.num_consecutive_trigs)
    #                         self.sensor0_trig_states.pop()
    #                 elif(j == 1):   # sensor1
    #                     self.sensor1_trig_states.insert(0, self.sensor_handler.get_sample(j, current_index).trig_state.name)    # add sensor1's trig status to list
    #                     print("sensor1_trig_states:", self.sensor1_trig_states)
    #                     if(len(self.sensor1_trig_states) >= self.num_consecutive_trigs):  # keep FIFO list with size of trigs required to change state (self.num_consecutive_trigs)
    #                         self.sensor1_trig_states.pop()
        
    #         # evaluate if last number of trig states stored in a separate list for each sensor have the same values  
    #         if all(x == self.sensor0_trig_states[0] for x in self.sensor0_trig_states) and all(y == self.sensor1_trig_states[0] for y in self.sensor1_trig_states):
    #             print("ready to change state")
    #             trig_states_are_same = True
    #             if self.init_idle_state_enabled:    # check if program runs first time when no state has been assigned 
    #                 if self.sensor0_trig_states[0] == SensorTrigState.NO_TRIG and self.sensor1_trig_states[0] == SensorTrigState.NO_TRIG:   # check if both sensors are in NO_TRIG state, to prevent changing to Idle if any of the sensors are blocked (in TRIG state) 
    #                     self.init_idle_state_enabled = False    # initialization phase passed
    #                     self.current_state.change_state(self.sensor0_trig_states[0], self.sensor1_trig_states[0], self.sensor_handler.get_sample(0, current_index).timestamp, trig_states_are_same)
    #             else:
    #                 self.current_state.change_state(self.sensor0_trig_states[0], self.sensor1_trig_states[0], self.sensor_handler.get_sample(0, current_index).timestamp, trig_states_are_same)
    #         else:
    #             print("NOT ready to change state")
    #             trig_states_are_same = False
    #             self.current_state.change_state(self.sensor0_trig_states[0], self.sensor1_trig_states[0], self.sensor_handler.get_sample(0, current_index).timestamp, trig_states_are_same)
            
                # provide the state itself with trig_state, timestamp and flag telling if sensor trig states were same for all trigs. then the state itself will handle to which state it will change to based on that

                # store trig_states in two separate lists (one for each sensor) and run below code to check if all trig values in each list are the same
                # add enabler to only check the lists current_index is larger than than num_consecutive_trigs
                # if(self.change_state_enabler == True):
                #     if(all(item1 == self.sensor0_trig_states[0] for item1 in self.sensor0_trig_states) and all(item2 == self.sensor1_trig_states[0] for item2 in self.sensor1_trig_states)):
                #         print(self.sensor0_trig_states)
                #         print(self.sensor1_trig_states)
                #         self.current_state.change_state(timestamp)
        
            
            

"""  
# create a state machine to handle the transitions to the states in which the program can be in 
class VisitorCounter:
    state = None    # initial state of the state machine
    
    def __init__(self, sensors, sensor_handler: SensorHandler, num_trig_threshold: int, num_false_trig_threshold: int, time_diff_threshold: int):
        self.sensors = sensors
        self.sensor_handler = sensor_handler
        self.num_trig_threshold = num_trig_threshold
        self.num_false_trig_threshold = num_false_trig_threshold
        #self.time_diff_threshold = time_diff_threshold
        self.sensor_dict = OrderedDict()   # dict of type OrderedDict in where the added items are stored in the order they are added
        self.current_sensor_trig_state = [] # list to store trig status of each of the sensors
        self.exit_trig_state_counter = 0
        self.entry_trig_state_counter = 0
        self.current_state = IdleState()
        
    def get_current_sensor_trig_state(self, index: int):
        # store current sensor readouts in dictionary to check which sensor that was trigged first
        self.current_sensor_trig_state.clear()  # clear list to only contain latest sensor trig states
        for sensor in range(len(self.sensors)):
            self.sensor_dict["sensor{0}".format(sensor)] = self.sensor_handler.get_sample(sensor, index)   # creates sensor0, sensor1, .. as keys (items stored in same order as they were added to the dict) with SensorHandler object as key
        #print(self.sensor_dict) 
        
        # identify current trig status for the sensors
        for key, value in self.sensor_dict.items():
            self.current_sensor_trig_state.append(value.trig_state)    # populate list with current sensor trig status, index value is equal to sensor id
        
        # print(self.sensor_dict["sensor0"].value)
        # print(self.sensor_dict["sensor0"].timestamp)
        # print(self.sensor_dict["sensor0"].trig_state)    
        return self.current_sensor_trig_state
    
    def detect_movement_direction(self, index: int):
        # initial sensor trig state readout
        sensor_trig_state = self.get_current_sensor_trig_state(index)   # get list indicating which sensors that are currently trigged
        
        # below code written based on two sensors
        # identify which sensor that was trigged first
        if sensor_trig_state[0] == SensorTrigState.TRIG and sensor_trig_state[1] == SensorTrigState.NO_TRIG:
            # check if enough number of positive sensor trigs have been reached
            if self.exit_trig_state_counter >= self.num_trig_threshold:
                self.exit_trig_state_counter = 0    # reset both entry and exit counters
                self.entry_trig_state_counter = 0
                print("initial EXIT movement detected")
                return ExitStateA()     # initial movement towards exit has been detected, switch state
            
            self.entry_trig_state_counter = 0   # reset opposite movement detection counter
            self.exit_trig_state_counter += 1   # increase positive trig counter
                
        elif sensor_trig_state[0] == SensorTrigState.NO_TRIG and sensor_trig_state[1] == SensorTrigState.TRIG:
            # check if enough number of positive sensor trigs have been reached
            if self.entry_trig_state_counter >= self.num_trig_threshold:
                self.exit_trig_state_counter = 0    # reset both entry and exit counters
                self.entry_trig_state_counter = 0
                print("initial ENTRY movement detected")
                return EntryStateA()    # initial movement towards entry has been detected, switch state
            
            self.exit_trig_state_counter = 0    # reset opposite movement detection counter
            self.entry_trig_state_counter += 1  # increase positive trig counter
        
        return self.exit_trig_state_counter, self.entry_trig_state_counter    
"""
        
def main():
    current_readout_index = 0
    number_of_sensors = 2
    max_samples = 10
    sensor_trig_threshold = 800     # sensor digital value (0 - 1023) to represent IR-sensor detection, below threshold value == sensor trig
    readout_frequency = 10  # Hz
    num_consecutive_trigs = 5
    num_consecutive_false_trigs = 5
    #time_diff_threshold = 0.2
    
    sensors = np.array([IrSensor(sensor_id, sensor_trig_threshold) for sensor_id in range(number_of_sensors)]) # create the rows in matrix that represents each of the sensors
    #print(sensors)
    sensor_handler = SensorHandler(number_of_sensors, max_samples)
    sensor_state_manager = SensorStateManager(number_of_sensors, sensor_handler, num_consecutive_trigs, num_consecutive_false_trigs)
    
    print("--before--")
    for i in range(max_samples):
        for j in range(number_of_sensors):
            print("[{:d}][{:d}] {:d} : {:d} : {:s}".format(j, i, sensor_handler.get_sample(j, i).value, sensor_handler.get_sample(j, i).timestamp, sensor_handler.get_sample(j, i).trig_state.name))
    print("-----")
    
    #while(True):
    for _ in range(15):
        for sensor_id, sensor in enumerate(sensors):
            #sensor_handle.register_sample(sensor_id, current_readout_index, *sensor.get_sensor_data())
            sensor_handler.register_sample(sensor_id, current_readout_index, *sensor.get_sensor_data())    # '*' unpacks the return tuple from function call

            # sensor_sample = sensor_handler.get_sample(sensor_id, 0)  # fetch current sensor readouts only
            # sensor_state_manager.evaluate_sensor_trig_states(sensor_sample.value, sensor_sample.timestamp)
        
        print("current_readout_index:", current_readout_index)
        sensor_state_manager.evaluate_sensor_trig_states(current_readout_index)
            

        if(current_readout_index == 3):
            print("sensor: 0; index: 3; value: {:d}; time: {:d}; state: {:s}".format(sensor_handler.get_sample(0, 3).value, sensor_handler.get_sample(0, 3).timestamp, sensor_handler.get_sample(0, 3).trig_state.name))
            print("sensor: 1; index: 3; value: {:d}; time: {:d}; state: {:s}".format(sensor_handler.get_sample(1, 3).value, sensor_handler.get_sample(1, 3).timestamp, sensor_handler.get_sample(1, 3).trig_state.name))
        
        current_readout_index += 1  # increase index to where store sensor readouts in the buffer
        
        if(current_readout_index >= max_samples):
            current_readout_index = 0   # when maximum buffer index is reached reset it to overwrite old data (FIFO)
        
        time.sleep(1/readout_frequency) # setting periodic time for sensor readout
        
    print("--after--")
    for i in range(max_samples):
        for j in range(number_of_sensors):
            print("[{:d}][{:d}] {:d} : {:d} : {:s}".format(j, i, sensor_handler.get_sample(j, i).value, sensor_handler.get_sample(j, i).timestamp, sensor_handler.get_sample(j, i).trig_state.name))
    print("-----")
    
    test = sensor_handler.get_sensor_logs()
    # print(type(test))
    print(test.shape)
    # print(test)
    
    
        
if __name__ == "__main__":
   main()