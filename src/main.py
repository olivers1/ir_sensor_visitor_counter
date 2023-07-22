import numpy as np
from enum import Enum
from collections import OrderedDict

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


# class SensorHandler:
#     def __init__(self, number_of_sensors: int, max_samples: int):
#         self.number_of_sensors = number_of_sensors
#         self.max_samples = max_samples
#         #self.sensor_logs = np.array([[SensorSample()]*max_samples] * self.number_of_sensors)    # create number of objects needed to store the buffered sensor data
#         self.sensor_logs = np.array([[SensorSample() for _ in range(max_samples)] for _ in range(self.number_of_sensors)])    # create number of objects needed to store the buffered sensor data
        
#     def register_sample(self, sensor_id, index, value, timestamp):
#         self.sensor_logs[sensor_id][index].set(value, timestamp)    
    
#     def get_sample(self, sensor_id, index):
#         return self.sensor_logs[sensor_id][index]
    
#     def get_sensor_logs(self):
#         return self.sensor_logs

class SensorTrigState(Enum):
        NO_TRIG = 0
        TRIG = 1


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


# states = {
#     "IDLE" : Idle(),
#     "A_OUT" = A_Out(),
#     "B_OUT" = B_Out(),
#     "C_OUT" = C_Out(),
#     "OUT_DETECTED" = OutDetected(),
#     "A_IN" = A_In(),
#     "B_IN" = B_In(),
#     "C_IN" = C_In(),
#     "IN_DETECTED" = InDetected()
#     }

    
# create a state machine to handle the transitions to the states in which the program can be in 
class VisitorCounter:
    state = None    # initial state of the state machine
    
    def __init__(self, sensors, index: int, sensor_handler: SensorHandler, num_trig_threshold: int, num_false_trig_threshold: int, time_diff_threshold: int):
        self.index = index
        self.sensor_handler = sensor_handler
        self.num_trig_threshold = num_trig_threshold
        self.num_false_trig_threshold = num_false_trig_threshold
        self.time_diff_threshold = time_diff_threshold
        self.sensor_dict = OrderedDict()   # dict of type OrderedDict where items are stored in the order they are added
        for sensor in enumerate(sensors):
            self.sensor_dict["sensor{0}".format(sensor)] = sensor  # creates sensor0, sensor1, .. variables stored in same order as they were added to the dict
            
    def get_first_sensor_trig(self):
        print(self.sensor_dict.keys())
        
        
        # for key, value in self.sensor_dict:
        #     test = value.value
        #     print(test)
            #key = self.sensor_handler.get_sample_trig_state(self.sensor_dict[key], self.index)  # returns a tuple (trig_state, timestamp)
            #print("key:", key)
        #sensor1 = self.sensor_handler.get_sample_trig_state(Idle.sensor1, self.index)  # returns a tuple (trig_state, timestamp)
        
            
        
class Idle:
    pass

class A_Out:
    pass

class B_Out:
    pass

class C_Out:
    pass

class OutDetected:
    pass



# class Idle:
#     # class attributes (global variables within a class)
#     sensor0 = 0 
#     sensor1 = 1
    
#     def __init__(self, index: int, sensor_handler: SensorHandler, num_trig_threshold: int, num_false_trig_threshold: int, time_diff_threshold: int):
#         self.index = index
#         self.sensor_handler = sensor_handler
#         self.num_trig_threshold = num_trig_threshold
#         self.num_false_trig_threshold = num_false_trig_threshold
#         self.time_diff_threshold = time_diff_threshold
        
#     def get_first_sensor_trig(self):
#         sensor0 = self.sensor_handler.get_sample_trig_state(Idle.sensor0, self.index)  # returns a tuple (trig_state, timestamp)
#         sensor1 = self.sensor_handler.get_sample_trig_state(Idle.sensor1, self.index)  # returns a tuple (trig_state, timestamp)
        
#         # check trig state for sensors
#         # ---| FUNCTION DESCRIPTION |---
#         # run until first trig is detected. when a trig for any sensor is detected, the function must return trig_state and timestamp
#         # maybe this part of function should be implemented in the IrSensor class instead
#         # the timestamp and trig state change will be used to determine which sensor that was trigged first
        
               

        
def main():
    current_readout_index = 0
    number_of_sensors = 2
    max_samples = 10
    sensor_trig_threshold = 800     # sensor digital value (0 - 1023) to represent IR-sensor detection, below threshold value == sensor trig
    readout_frequency = 10  # in Hz
    num_trig_threshold = 5
    num_false_trig_threshold = 5
    time_diff_threshold = 0.2
    
    sensors = np.array([IrSensor(sensor_id, sensor_trig_threshold) for sensor_id in range(number_of_sensors)]) # create the rows in matrix that represents the sensors
    #print(sensors)
    sensor_handler = SensorHandler(number_of_sensors, max_samples)
    visitor_counter = VisitorCounter(sensors, current_readout_index, sensor_handler, num_trig_threshold, num_false_trig_threshold, time_diff_threshold)
    visitor_counter.get_first_sensor_trig()
    
    #while(True):
    for _ in range(15):
        for sensor_id, sensor in enumerate(sensors):
            #sensor_handle.register_sample(sensor_id, current_readout_index, *sensor.get_sensor_data())
            sensor_handler.register_sample(sensor_id, current_readout_index, *sensor.get_sensor_data())    # '*' unpacks the return tuple from function call
        
        print("current_readout_index:", current_readout_index)    
        
        if(current_readout_index == 3):
            print("sensor: 0; index: 3; value: {:d}; time: {:d}; state: {:s}".format(sensor_handler.get_sample(0, 3).value, sensor_handler.get_sample(0, 3).timestamp, sensor_handler.get_sample(0, 3).trig_state.name))
            print("sensor: 1; index: 3; value: {:d}; time: {:d}; state: {:s}".format(sensor_handler.get_sample(1, 3).value, sensor_handler.get_sample(1, 3).timestamp, sensor_handler.get_sample(1, 3).trig_state.name))
        
        current_readout_index += 1  # increase index to where store sensor readouts in the buffer
        
        if(current_readout_index >= max_samples):
            current_readout_index = 0   # when maximum buffer index is reached reset it to overwrite old data (FIFO)
        
        time.sleep(1/readout_frequency) # setting periodic time for sensor readout
        
    print("-----")
    for i in range(max_samples):
        for j in range(number_of_sensors):
            print("[{:d}][{:d}] {:d} : {:d}".format(j, i, sensor_handler.get_sample(j, i).value, sensor_handler.get_sample(j, i).timestamp))
    print("-----")
    
    test = sensor_handler.get_sensor_logs()
    print(type(test))
    print(test.shape)
    print(test)
    
    
        
if __name__ == "__main__":
   main()