import numpy as np
from enum import Enum
from abc import ABC, abstractmethod
from inspect import signature

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
    
    def get_sample(self, sensor_id: int, index: int):
        return self.sensor_logs[sensor_id][index]
    
    def get_sensor_logs(self):
        return self.sensor_logs
    

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

class SensorStateManager:
    current_state = MotionDetectionState.INIT   # hold current state
    succeeding_state = MotionDetectionState.IDLE    # next state to which the program will transition to based on current sensor trig states
   
    def __init__(self, number_of_sensors: int, sensor_handler: SensorHandler, num_consecutive_trigs: int, num_consecutive_unwanted_state_changes: int):
        self.number_of_sensors = number_of_sensors
        self.sensor_handler = sensor_handler
        self.num_consecutive_trigs = num_consecutive_trigs
        self.num_consecutive_unwanted_state_changes = num_consecutive_unwanted_state_changes    # amount of unwanted state changes when current state is where a entry/exit motion detection has been initiated (not in INIT or IDLE state)
        self.unwanted_state_change_counter = 0     # counter to keep track of number of unwanted stage changes to determine when program should go back to INIT state
        self.sensor_trig_states = np.array([[SensorTrigState.NO_TRIG for _ in range(num_consecutive_trigs)] for _ in range(self.number_of_sensors)])    # create number of items needed a buffered number of sensor trig states for evaluation
        self.change_state_is_allowed = False    # enabler to start evaluate sensor trig states only after a specified number of sensor readouts
        self.sample_counter = 0     # index to keep track of where in sensor_trig_states array to store sensor trig states
        self.state_change_timestamp = 0
        self.transition_table = self.get_transition_dict()  # dictionary with integer value as keys and MotionDetectionState enum as values
        
    def get_transition_key(self, sensor0_trig_state: SensorTrigState, sensor1_trig_state: SensorTrigState, current_state: MotionDetectionState, change_to_state: MotionDetectionState):
            # calculate an unique number based on current state parameters
            sum = sensor0_trig_state.value *1 + sensor1_trig_state.value *10 + current_state.value *100 + change_to_state.value *0  # succeeding state used for covenience only to create the transition dictionary from same list
            return sum
        
    def get_transition_dict(self):
        transition_states = [
            # sensor0_trig_state.value, sensor1_trig_state.value, current_state.value, succeeding_state.value
            # INIT
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.INIT, MotionDetectionState.IDLE,
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.INIT, MotionDetectionState.INIT,        # allow stay in same state
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.INIT, MotionDetectionState.INIT,        # allow stay in same state
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.INIT, MotionDetectionState.INIT,           # allow stay in same state
            
            # IDLE
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.IDLE, MotionDetectionState.IDLE,     # allow stay in current state with sensors at same trig states that got the program to the state
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.IDLE, MotionDetectionState.IDLE,           # allow stay in same state
            # -EXIT
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.IDLE, MotionDetectionState.EXIT_A,
            # -ENTRY
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.IDLE, MotionDetectionState.ENTRY_A,
            
            # EXIT
            # -EXIT_A
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.EXIT_A, MotionDetectionState.EXIT_B,
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.EXIT_A, MotionDetectionState.EXIT_A,    # allow stay in same state
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.EXIT_A, MotionDetectionState.IDLE,
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.EXIT_A, MotionDetectionState.IDLE,
            # -EXIT_B
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.EXIT_B, MotionDetectionState.EXIT_C,
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.EXIT_B, MotionDetectionState.EXIT_B,       # allow stay in same state
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.EXIT_B, MotionDetectionState.EXIT_A,
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.EXIT_B, MotionDetectionState.EXIT_A,
            # -EXIT_C
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.EXIT_C, MotionDetectionState.EXIT_COMPLETE,
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.EXIT_C, MotionDetectionState.EXIT_C,    # allow stay in same state
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.EXIT_C, MotionDetectionState.EXIT_B,
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.EXIT_C, MotionDetectionState.EXIT_B,
            # -EXIT_COMPLETE
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.EXIT_COMPLETE, MotionDetectionState.IDLE,
            
            # ENTRY
            # -ENTRY_A
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.ENTRY_A, MotionDetectionState.ENTRY_B,
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.ENTRY_A, MotionDetectionState.ENTRY_A,  # allow stay in same state
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.ENTRY_A, MotionDetectionState.IDLE,
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.ENTRY_A, MotionDetectionState.IDLE,
            # -ENTRY_B
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.ENTRY_B, MotionDetectionState.ENTRY_C,
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.ENTRY_B, MotionDetectionState.ENTRY_B,     # allow stay in same state
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.ENTRY_B, MotionDetectionState.ENTRY_A,
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.ENTRY_B, MotionDetectionState.ENTRY_A,
            # -ENTRY_C
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.ENTRY_C, MotionDetectionState.ENTRY_COMPLETE,
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.ENTRY_C, MotionDetectionState.ENTRY_C,  # allow stay in same state
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.ENTRY_C, MotionDetectionState.ENTRY_B,
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.ENTRY_C, MotionDetectionState.ENTRY_B,
            # -ENTRY_COMPLETE
            SensorTrigState.NO_TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.ENTRY_COMPLETE, MotionDetectionState.IDLE,
        ]
        
        transition_keys = []    # stores the keys for the unique states that will be used to create a transition table dictionary
        transition_values = []  # stores the values for succeeding state change after current state
        transition_table = {}
        num_transition_table_parameters = len(signature(self.get_transition_key).parameters)    # number of parameters the function takes to use as step in for loop
        for i in range(0, len(transition_states), num_transition_table_parameters):     # loop through list containing all allowed transitions
            arguments = transition_states[i:i + num_transition_table_parameters]
            transition_keys.append(self.get_transition_key(*arguments))     # split list into its items to be used as arguments for function call
            transition_values.append(MotionDetectionState(arguments[-1]))   # extract last item in sliced list which contains the succeeding state change after current state
        
        for key, value in zip(transition_keys, transition_values):  # populate dictionary
            transition_table[key] = value
        return transition_table

    def import_sensor_trig_states(self, current_readout_index: int):
        #for i in range(self.num_consecutive_trigs):   # use current sensor readout index get latest sensor trig state as well as a number of previous readouts specified by num_consecutive_trigs variable
        for sensor_id in range(self.number_of_sensors):
            self.sensor_trig_states[sensor_id][self.sample_counter] = self.sensor_handler.get_sample(sensor_id, current_readout_index).trig_state
        
        self.sample_counter += 1
        print("sample_counter:", self.sample_counter)
        # keep sensor_trig_states buffer at fixed size
        if self.sample_counter >= self.num_consecutive_trigs:   
            self.sample_counter = 0
        #print(self.sensor_trig_states)
    
    def verify_sensor_trig_continuity(self):
        verified_trig_states = []   # list containing the verified trig states for sensor_id identified by its index in list
        for sensor_id in range(self.number_of_sensors):
            ref_item = self.sensor_trig_states[sensor_id][0]    # reference value to compare with the other list items
            all_elements_are_same = True
            for item in self.sensor_trig_states[sensor_id]:
                if ref_item != item:
                    all_elements_are_same = False   # set boolean to False if any of the trig_states in the list deviates from the other
                    break
            if all_elements_are_same:   
                verified_trig_states.append(self.sensor_trig_states[sensor_id][0])  # if alla items are same add that sensor trig state
            else:
                verified_trig_states.append(SensorTrigState.UNKNOWN)   # if not all items are same add 'UNKNOWN' trig state to list
        return verified_trig_states
        
    def detect_motion_direction(self, sensor_states, current_readout_index):
        # calculate unique value based on current state and sensor trig states were program is executing at now
        # formula used to calculate current state value
        # sensor0_trig_state.value *1 + sensor1_trig_state.value *10 + self.current_state *100
        current_state_sum = 0   # holds the unique value of the sensor trig states and current state
        for sensor_id in range(self.number_of_sensors):
            if sensor_id == 1:  # check which sensor being assigned a value since sensor1 trig state value is multiplied by 10
                current_state_sum += self.sensor_trig_states[sensor_id][self.sample_counter].value * 10     # sensor1
            else:
                current_state_sum += self.sensor_trig_states[sensor_id][self.sample_counter].value          # sensor0
        
        current_state_sum += self.current_state.value * 100     # add current state value to sum
        print("current_state_sum:", current_state_sum)
        # check if state current state sum exists in dictionary and add succeeding state to variable
        succeeding_state_value = self.transition_table.get(current_state_sum, -1)   # -1 is returned if key not found in dictionary
        self.succeeding_state = MotionDetectionState(succeeding_state_value)
        
        if self.succeeding_state != self.current_state:     # check if a state change will be performed
            self.sensor_trig_states.fill(0)     # clear trig_state array at every state change to avoid falling back several states at once when unwanted state change is detected
        
        # perform state change and add timestamp when state is changed
        timestamp = 0   # holds the timestamp when a state change occured
        if self.succeeding_state.value > self.current_state.value:   # identify state change direction (moving direction) by its enum values
            self.current_state = self.succeeding_state      # change state
            timestamp = self.state_change_timestamp = self.sensor_handler.get_sample(0, current_readout_index).timestamp  # get sensor timestamp from last sensor readout
        elif self.succeeding_state.value < self.current_state.value:
            if self.succeeding_state != MotionDetectionState.IDLE:  # this will handle the transition from EXIT_COMPLETE and ENTRY_COMPLETE to IDLE state
                self.unwanted_state_change_counter += 1     # increase counter
            else:
                self.current_state = self.succeeding_state      # change state, complete motion captured back to IDLE state
                timestamp = self.state_change_timestamp = self.sensor_handler.get_sample(0, current_readout_index).timestamp  # get sensor timestamp from last sensor readout
        print("unwanted_state_change_counter:", self.unwanted_state_change_counter)
        
        if self.unwanted_state_change_counter >= self.num_consecutive_unwanted_state_changes:
            self.unwanted_state_change_counter = 0      # reset counter
            self.current_state = MotionDetectionState(self.current_state.value - 1)   # number of unwanted trig states reached, change to previous state 
            timestamp = self.state_change_timestamp = self.sensor_handler.get_sample(0, current_readout_index).timestamp  # get sensor timestamp from last sensor readout
            
        if self.current_state == MotionDetectionState.IDLE:
            self.unwanted_state_change_counter = 0      # reset counter. IDLE is the base state, after INIT is fulfilled once it will never go back to this state
         
        return self.current_state.name, timestamp
        
    
    def evaluate_sensor_trig_states(self, current_readout_index: int):
        self.import_sensor_trig_states(current_readout_index)
        
        if self.sample_counter >= self.num_consecutive_trigs -1: # enable state change only after a specified number of sensor readouts
            self.change_state_is_allowed = True
        
        if self.change_state_is_allowed:    # enough number of sensor sample readouts performed to be able evalute sensor trig states
            verified_sensor_states = self.verify_sensor_trig_continuity()
            print(verified_sensor_states)
            
            # check if verified sensor states are stable before performing any state change
            sensor_states_are_stable = True
            for sensor_state in verified_sensor_states:
                if(sensor_state) == SensorTrigState.UNKNOWN:
                    sensor_states_are_stable = False
                    break
            if sensor_states_are_stable:
                state_change = self.detect_motion_direction(verified_sensor_states, current_readout_index)
                print(state_change)

        
def main():
    current_readout_index = 0
    number_of_sensors = 2
    max_samples = 10
    sensor_trig_threshold = 800     # sensor digital value (0 - 1023) to represent IR-sensor detection, below threshold value == sensor trig
    readout_frequency = 20  # Hz
    num_consecutive_trigs = 5
    num_consecutive_unwanted_state_changes = 3
    #time_diff_threshold = 0.2
    
    sensors = np.array([IrSensor(sensor_id, sensor_trig_threshold) for sensor_id in range(number_of_sensors)]) # create the rows in matrix that represents each of the sensors
    #print(sensors)
    sensor_handler = SensorHandler(number_of_sensors, max_samples)
    sensor_state_manager = SensorStateManager(number_of_sensors, sensor_handler, num_consecutive_trigs, num_consecutive_unwanted_state_changes)
    
    # print("--before--")
    # for i in range(max_samples):
    #     for j in range(number_of_sensors):
    #         print("[{:d}][{:d}] {:d} : {:d} : {:s}".format(j, i, sensor_handler.get_sample(j, i).value, sensor_handler.get_sample(j, i).timestamp, sensor_handler.get_sample(j, i).trig_state.name))
    # print("-----")
    
    while(True):
    #for _ in range(15):
        for sensor_id, sensor in enumerate(sensors):
            sensor_handler.register_sample(sensor_id, current_readout_index, *sensor.get_sensor_data())    # '*' unpacks the return tuple from function call

            # sensor_sample = sensor_handler.get_sample(sensor_id, 0)  # fetch current sensor readouts only
            # sensor_state_manager.evaluate_sensor_trig_states(sensor_sample.value, sensor_sample.timestamp)
        
        print("current_readout_index:", current_readout_index)
        sensor_state_manager.evaluate_sensor_trig_states(current_readout_index)

        # if(current_readout_index == 3):
        #     print("sensor: 0; index: 3; value: {:d}; time: {:d}; state: {:s}".format(sensor_handler.get_sample(0, 3).value, sensor_handler.get_sample(0, 3).timestamp, sensor_handler.get_sample(0, 3).trig_state.name))
        #     print("sensor: 1; index: 3; value: {:d}; time: {:d}; state: {:s}".format(sensor_handler.get_sample(1, 3).value, sensor_handler.get_sample(1, 3).timestamp, sensor_handler.get_sample(1, 3).trig_state.name))
        
        current_readout_index += 1  # increase index to where store sensor readouts in the buffer
        
        if(current_readout_index >= max_samples):
            current_readout_index = 0   # when maximum buffer index is reached reset it to overwrite old data (FIFO)
        
        time.sleep(1/readout_frequency) # setting periodic time for sensor readout
        
    print("--after--")
    for i in range(max_samples):
        for j in range(number_of_sensors):
            print("[{:d}][{:d}] {:d} : {:d} : {:s}".format(j, i, sensor_handler.get_sample(j, i).value, sensor_handler.get_sample(j, i).timestamp, sensor_handler.get_sample(j, i).trig_state.name))
    print("-----")
    
    sensor_logs = sensor_handler.get_sensor_logs()
    # print(type(sensor_logs))
    print(sensor_logs.shape)
    # print(sensor_logs)

    
        
if __name__ == "__main__":
   main()