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
    

class Idle(SensorState):
    def __init__(self, timestamp: int):
        self.last_updated = timestamp
        
    def change_state(self, timestamp: int):
        print("hello world!")
    
    def reset_state(self):
        print("hello initstate")


class Exit_0():
    pass


class Entry_0():
    pass


class SensorStateManager:
    current_state: SensorState = Idle
    
    def __init__(self, number_of_sensors: int, sensor_handler: SensorHandler, num_positive_trig_threshold: int, num_negative_trig_threshold: int):
        self.number_of_sensors = number_of_sensors
        self.sensor_handler = sensor_handler
        self.num_positive_trig_threshold = num_positive_trig_threshold
        self.num_negative_trig_threshold = num_negative_trig_threshold

        self.change_state_enabler = False
        
    def evaluate_sensor_trig_states(self, current_index: int):
        for current_index in range(current_index, current_index - self.num_positive_trig_threshold, -1):   # check a number of latest consecutive sensor trig status readouts (number specified by num_positive_trig_threshold) individually, for each sensor have the same trig status
            for j in range(self.number_of_sensors):
                print("[{:d}][{:d}[{:s}]".format(j, current_index, self.sensor_handler.get_sample(j, current_index).trig_state.name))
        
        # write code here to compare check if each individual sensor has num_positive_trig_threshold consecutive same values this means the sensor is stable and requirement has been fulfilled to allow switch state

            
        
            
            

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
    num_positive_trig_threshold = 5
    num_negative_trig_threshold = 5
    #time_diff_threshold = 0.2
    
    sensors = np.array([IrSensor(sensor_id, sensor_trig_threshold) for sensor_id in range(number_of_sensors)]) # create the rows in matrix that represents each of the sensors
    #print(sensors)
    sensor_handler = SensorHandler(number_of_sensors, max_samples)
    sensor_state_manager = SensorStateManager(number_of_sensors, sensor_handler, num_positive_trig_threshold, num_negative_trig_threshold)
    
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