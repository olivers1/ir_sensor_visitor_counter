import numpy as np
from enum import Enum
from abc import ABC, abstractmethod
from inspect import signature
import datetime
import pyrebase

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import time
import RPi.GPIO as GPIO


# Setup firebase web tool to send data to it
config = {
    "apiKey" : "wxmm8jpqkGlyfTX8kOGOQ1gf2ItvjySFcJyyCGxP",
    "authDomain" : "ir-sensor-cat-counter.firebaseapp.com",
    "databaseURL" : "https://ir-sensor-cat-counter-default-rtdb.europe-west1.firebasedatabase.app/",
    "storageBucket" : "ir-sensor-cat-counter.appspot.com"
}
firebase = pyrebase.initialize_app(config)
db = firebase.database()

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
    

class TransitionTableProvider:
    def __init__(self):
        pass
        
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
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.EXIT_COMPLETE, MotionDetectionState.EXIT_COMPLETE,  # allow stay in same state
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.EXIT_COMPLETE, MotionDetectionState.EXIT_COMPLETE,  # allow stay in same state
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.EXIT_COMPLETE, MotionDetectionState.EXIT_COMPLETE,     # allow stay in same state
            
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
            SensorTrigState.TRIG, SensorTrigState.NO_TRIG, MotionDetectionState.ENTRY_COMPLETE, MotionDetectionState.ENTRY_COMPLETE,    # allow stay in same state
            SensorTrigState.NO_TRIG, SensorTrigState.TRIG, MotionDetectionState.ENTRY_COMPLETE, MotionDetectionState.ENTRY_COMPLETE,    # allow stay in same state
            SensorTrigState.TRIG, SensorTrigState.TRIG, MotionDetectionState.ENTRY_COMPLETE, MotionDetectionState.ENTRY_COMPLETE,       # allow stay in same state
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
            
    def get_transition_table(self):
        transition_table = self.get_transition_dict()   # generate transition table
        return transition_table

class SensorStateManager:
    current_state = MotionDetectionState.INIT   # hold current state
    succeeding_state = MotionDetectionState.IDLE    # next state to which the program will transition to based on current sensor trig states
    state_is_changed = False
   
    def __init__(self, number_of_sensors: int, sensor_handler: SensorHandler, transition_table: TransitionTableProvider, num_consecutive_trigs: int, num_consecutive_unwanted_state_changes: int, log_file_name: str):
        self.number_of_sensors = number_of_sensors
        self.sensor_handler = sensor_handler
        self.num_consecutive_trigs = num_consecutive_trigs
        self.num_consecutive_unwanted_state_changes = num_consecutive_unwanted_state_changes    # amount of unwanted state changes when current state is where a entry/exit motion detection has been initiated (not in INIT or IDLE state)
        self.filename = log_file_name   # log file to store state changes locally
        self.unwanted_state_change_counter = 0     # counter to keep track of number of unwanted stage changes to determine when program should go back to INIT state
        self.sensor_trig_states = np.array([[SensorTrigState.NO_TRIG for _ in range(num_consecutive_trigs)] for _ in range(self.number_of_sensors)])    # create number of items needed a buffered number of sensor trig states for evaluation
        self.change_state_is_allowed = False    # enabler to start evaluate sensor trig states only after a specified number of sensor readouts
        self.sample_counter = 0     # index to keep track of where in sensor_trig_states array to store sensor trig states
        self.transition_table = transition_table.get_transition_table()     # dictionary with integer value as keys and MotionDetectionState enum as values
        print("transition_table type:", type(self.transition_table))    # import transition table dictionary from TransitionTableProvider class
        self.verified_sensor_trig_states = []  # list containing the verified trig states for sensor_id identified by its index in list

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
        self.verified_sensor_trig_states.clear()    # clear list too keep it from growing
        for sensor_id in range(self.number_of_sensors):
            ref_item = self.sensor_trig_states[sensor_id][0]    # reference value to compare with the other list items
            all_elements_are_same = True
            for item in self.sensor_trig_states[sensor_id]:
                if ref_item != item:
                    all_elements_are_same = False   # set boolean to False if any of the trig_states in the list deviates from the other
                    break
            
            if all_elements_are_same:   
                self.verified_sensor_trig_states.append(self.sensor_trig_states[sensor_id][0])      # if alla items are same add that sensor trig state
            else:
                self.verified_sensor_trig_states.append(SensorTrigState.UNKNOWN)    # if not all list items are same add 'UNKNOWN' trig state to list
        print("verified_sensor_trig_states:", self.verified_sensor_trig_states)
        return all_elements_are_same    # returns True if all list items in each of the two lists respectively are same
    
    def get_current_state_sum(self):
        # calculate unique value based on current state and sensor trig states which is currently present
        # sensor0_trig_state.value *1 + sensor1_trig_state.value *10 + self.current_state *100
        current_state_sum = 0   # holds the unique value of the sensor trig states and current state
        multiply = [1, 10]  # sensor id multiplication values
        for sensor_id in range(self.number_of_sensors):
            current_state_sum += self.sensor_trig_states[sensor_id][self.sample_counter].value * multiply[sensor_id]    # multiply sensorstate sum with specified value dependent on sensor id
            
        current_state_sum += self.current_state.value * 100     # add current state value to sum
        return current_state_sum
    
    def change_state(self, current_readout_index):
        # perform state change and add timestamp when state is changed
        timestamp = 0   # holds the timestamp when a state change occured
        if self.succeeding_state.value > self.current_state.value:   # identify state change direction (moving direction) by its enum values
            self.current_state = self.succeeding_state      # change state
            timestamp = self.sensor_handler.get_sample(0, current_readout_index).timestamp  # get sensor timestamp from last sensor readout
        elif self.succeeding_state.value < self.current_state.value:
            if self.succeeding_state != MotionDetectionState.IDLE:  # this will handle the transition from EXIT_COMPLETE and ENTRY_COMPLETE to IDLE state
                self.unwanted_state_change_counter += 1     # increase counter
            else:
                self.current_state = self.succeeding_state      # change state, complete motion captured back to IDLE state
                timestamp = self.sensor_handler.get_sample(0, current_readout_index).timestamp  # get sensor timestamp from last sensor readout
        print("unwanted_state_change_counter:", self.unwanted_state_change_counter)
        
        if self.unwanted_state_change_counter >= self.num_consecutive_unwanted_state_changes:
            self.unwanted_state_change_counter = 0      # reset counter
            self.current_state = MotionDetectionState(self.current_state.value - 1)   # number of unwanted trig states reached, change to previous state 
            timestamp = self.sensor_handler.get_sample(0, current_readout_index).timestamp  # get sensor timestamp from last sensor readout

        return timestamp
        
    def detect_motion_direction(self, current_readout_index):
        current_state_sum = self.get_current_state_sum()    # get the unique value representation of sensor trig states and current state where motion dection algorithm is current running
        print("current_state_sum:", current_state_sum)
        
        # check if state current state sum exists in dictionary and update succeeding state to variable
        succeeding_state_value = self.transition_table.get(current_state_sum, -1)   # -1 is returned if key not found in dictionary
        self.succeeding_state = MotionDetectionState(succeeding_state_value)
        
        if self.succeeding_state != self.current_state:     # check if a state change will be performed
            self.state_is_changed = True    #   state will be changed, used as enabler for logging a state change
            self.sensor_trig_states.fill(0)     # clear trig_state array at every state change to avoid falling back several states at once when unwanted state change is detected
        else:
            self.state_is_changed = False
        
        timestamp = self.change_state(current_readout_index)    # analyse if and what state change to perform
        
        if self.current_state == MotionDetectionState.IDLE:
            self.unwanted_state_change_counter = 0      # reset counter. IDLE is the base state, after INIT is fulfilled once it will never go back to this state
         
        return timestamp, self.current_state.name
    
    def log_state_change(self, state_change):
        # write to log at every state change instance only
        if self.state_is_changed:   # log the current state only when the state has been changed
            # write to local log file
            try:
                with open(self.filename, 'a') as file:
                    try:
                        file.write(str(state_change[0]) + ", " + str(state_change[1]) + "\n")
                    except (IOError, OSError):
                        print("Error writing to file")
            except (FileNotFoundError, PermissionError, OSError):
                print("Error opening file")
            
            # send data to firebase database
            data = {
            "Timestamp" : state_change[0],
            "State" : state_change[1]
            } 
            db.child("StateChange").push(data)
            db.update (data)
            #print ("data sent to firebase")
    
    def evaluate_sensor_trig_states(self, current_readout_index: int):
        self.import_sensor_trig_states(current_readout_index)
        
        if self.sample_counter >= self.num_consecutive_trigs -1: # enable state change only after a specified number of sensor readouts
            self.change_state_is_allowed = True
        
        if self.change_state_is_allowed:    # enough number of sensor sample readouts performed to be able evalute sensor trig states
            sensor_states_are_stable = self.verify_sensor_trig_continuity()     # True if all sensor trigs have same value in each of the lists
            
            if sensor_states_are_stable:
                state_change = self.detect_motion_direction(current_readout_index)
                
                # write to log file when every new state change occurs
                self.log_state_change(state_change)
                print(state_change)


        
def main():
    current_readout_index = 0
    number_of_sensors = 2
    max_samples = 10
    sensor_trig_threshold = 800     # sensor digital value (0 - 1023) to represent IR-sensor detection, below threshold value == sensor trig
    readout_frequency = 2  # Hz
    num_consecutive_trigs = 5
    num_consecutive_unwanted_state_changes = 3
    log_file_name = "/home/olivers/Documents/python/ir_sensor_visitor_counter/logs/" + datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%SZ")
    
    sensors = np.array([IrSensor(sensor_id, sensor_trig_threshold) for sensor_id in range(number_of_sensors)]) # create the rows in matrix that represents each of the sensors
    #print(sensors)
    sensor_handler = SensorHandler(number_of_sensors, max_samples)
    transition_table = TransitionTableProvider()
    sensor_state_manager = SensorStateManager(number_of_sensors, sensor_handler, transition_table, num_consecutive_trigs, num_consecutive_unwanted_state_changes, log_file_name)
    
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