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
    current_state: MotionDetectionState = MotionDetectionState.INIT
    
    def __init__(self, number_of_sensors: int, sensor_handler: SensorHandler, num_consecutive_trigs: int, num_consecutive_false_trigs: int):
        self.number_of_sensors = number_of_sensors
        self.sensor_handler = sensor_handler
        self.num_consecutive_trigs = num_consecutive_trigs
        #self.num_consecutive_false_trigs = num_consecutive_false_trigs
        self.sensor_trig_states = np.array([[SensorTrigState.NO_TRIG for _ in range(num_consecutive_trigs)] for _ in range(self.number_of_sensors)])    # create number of items needed a buffered number of sensor trig states for evaluation
        self.change_state_is_allowed = False    # enabler to start evaluate sensor trig states only after a specified number of sensor readouts
        self.sample_counter = 0     # index to keep track of where in sensor_trig_states array to store sensor trig states

    def import_sensor_trig_states(self, current_readout_index: int):
        #for i in range(self.num_consecutive_trigs):   # use current sensor readout index get latest sensor trig state as well as a number of previous readouts specified by num_consecutive_trigs variable
        for sensor_id in range(self.number_of_sensors):
            self.sensor_trig_states[sensor_id][self.sample_counter] = self.sensor_handler.get_sample(sensor_id, current_readout_index).trig_state.name
        # print("sensor_logs sensor0:", self.sensor_trig_states[0][self.sample_counter])
        # print("sensor_logs sensor1:", self.sensor_trig_states[1][self.sample_counter])
        print("sample_counter:", self.sample_counter)
        print(self.sensor_trig_states)
        self.sample_counter += 1
        if self.sample_counter >= self.num_consecutive_trigs:   # keep sensor_trig_states buffer at fixed size
            self.sample_counter = 0
    
    def verify_sensor_trig_continuity(self):
        verified_trig_states = []   # list containing the verified trig states for sensor_id identified by its index in list
        for sensor_id in range(self.number_of_sensors):
            ref_item = self.sensor_trig_states[sensor_id][0]    # reference value to compare with the other list items
            all_elements_are_same = True
            for item in self.sensor_trig_states[sensor_id]:
                if ref_item != item:
                    all_elements_are_same = False
                    
            if all_elements_are_same:   
                verified_trig_states.append(self.sensor_trig_states[sensor_id][0])  # if alla items are same add that sensor trig state
            else:
                verified_trig_states.append(SensorTrigState.UNKNOWN.name)   # if not all items are same add 'UNKNOWN' trig state to list
                
    #def change_state(self):
    #    pass
      
        
    def detect_motion_direction(self, sensor_states):
        # identify the sensor verified trig states to decide which state to change
        if sensor_states[0] == SensorTrigState.NO_TRIG.name and sensor_states[1] == SensorTrigState.NO_TRIG.name:   # check if both sensors are in verified NO_TRIG state
            if self.current_state == MotionDetectionState.INIT:
                self.current_state = MotionDetectionState.IDLE
                # add timestamp and/or print it somewhere when reaching this state
            elif self.current_state == MotionDetectionState.EXIT_C:
                self.current_state = MotionDetectionState.EXIT_COMPLETE
                # add timestamp and/or print it somewhere when reaching this state
            elif self.current_state == MotionDetectionState.ENTRY_C:
                self.current_state = MotionDetectionState.ENTRY_COMPLETE
                # add timestamp and/or print it somewhere when reaching this state
                
        elif self.current_state == MotionDetectionState.IDLE:
            if sensor_states[0] == SensorTrigState.TRIG.name and sensor_states[1] == SensorTrigState.NO_TRIG.name:
                self.current_state = MotionDetectionState.EXIT_A
            elif sensor_states[0] == SensorTrigState.NO_TRIG.name and sensor_states[1] == SensorTrigState.TRIG.name:
                self.current_state = MotionDetectionState.ENTRY_A
        
        elif self.current_state == MotionDetectionState.EXIT_A:
            if sensor_states[0] == SensorTrigState.TRIG.name and sensor_states[1] == SensorTrigState.TRIG.name:
                self.current_state = MotionDetectionState.EXIT_B
            else:
                self.current_state = MotionDetectionState.INIT
                # not sure yet how to handle if motion doesn't continue
                
        elif self.current_state == MotionDetectionState.EXIT_B:
            if sensor_states[0] == SensorTrigState.NO_TRIG.name and sensor_states[1] == SensorTrigState.TRIG.name:
                self.current_state = MotionDetectionState.EXIT_C
            else:
                self.current_state = MotionDetectionState.INIT
                # not sure yet how to handle if motion doesn't continue
        
        elif self.current_state == MotionDetectionState.ENTRY_A:
            if sensor_states[0] == SensorTrigState.TRIG.name and sensor_states[1] == SensorTrigState.TRIG.name:
                self.current_state = MotionDetectionState.ENTRY_B
            else:
                self.current_state = MotionDetectionState.INIT
                # not sure yet how to handle if motion doesn't continue
        
        elif self.current_state == MotionDetectionState.ENTRY_B:
            if sensor_states[0] == SensorTrigState.TRIG.name and sensor_states[1] == SensorTrigState.NO_TRIG.name:
                self.current_state = MotionDetectionState.ENTRY_C
            else:
                self.current_state = MotionDetectionState.INIT
                # not sure yet how to handle if motion doesn't continue
        print("current_state:", self.current_state.name)
    
    def evaluate_sensor_trig_states(self, current_readout_index: int):
        self.import_sensor_trig_states(current_readout_index)
        
        if current_readout_index >= self.num_consecutive_trigs -1: # enable state change only after a specified number of sensor readouts
            self.change_state_is_allowed = True
        
        if self.change_state_is_allowed:    # enough number of sensor sample readouts performed to be able evalute sensor trig states
            verified_sensor_states = self.verify_sensor_trig_continuity()
            print( verified_sensor_states)
            self.detect_motion_direction(verified_sensor_states)
       

        
def main():
    current_readout_index = 0
    number_of_sensors = 2
    max_samples = 10
    sensor_trig_threshold = 800     # sensor digital value (0 - 1023) to represent IR-sensor detection, below threshold value == sensor trig
    readout_frequency = 2  # Hz
    num_consecutive_trigs = 5
    num_consecutive_false_trigs = 5
    #time_diff_threshold = 0.2
    
    sensors = np.array([IrSensor(sensor_id, sensor_trig_threshold) for sensor_id in range(number_of_sensors)]) # create the rows in matrix that represents each of the sensors
    #print(sensors)
    sensor_handler = SensorHandler(number_of_sensors, max_samples)
    sensor_state_manager = SensorStateManager(number_of_sensors, sensor_handler, num_consecutive_trigs, num_consecutive_false_trigs)
    
    # print("--before--")
    # for i in range(max_samples):
    #     for j in range(number_of_sensors):
    #         print("[{:d}][{:d}] {:d} : {:d} : {:s}".format(j, i, sensor_handler.get_sample(j, i).value, sensor_handler.get_sample(j, i).timestamp, sensor_handler.get_sample(j, i).trig_state.name))
    # print("-----")
    
    #while(True):
    for _ in range(15):
        for sensor_id, sensor in enumerate(sensors):
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