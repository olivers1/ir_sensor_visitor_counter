import numpy as np

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
        self.value : int = 0
        self.timestamp : int = 0
        
    def set(self, value, timestamp):
        self.value = value
        self.timestamp = timestamp
    
    def get(self):
        return self.value, self.timestamp


class IrSensor:
    def __init__(self, mcp_channel: int):
        self.mcp_channel = mcp_channel
        
    def get_sensor_data(self):
        # read sensor value and timestamp and return it
        return mcp.read_adc(self.mcp_channel), round(time.time()*1000)


class SensorHandler:
    def __init__(self, number_of_sensors : int, max_samples :int):
        self.number_of_sensors = number_of_sensors
        self.max_samples = max_samples
        # NOT WORKING HERE, self.sensor_logs DOES NOT BECOME OF RIGHT TYPE (SensorSample)
        self.sensor_logs = [[SensorSample()]*max_samples] * self.number_of_sensors    # create number of objects needed to store the buffered sensor data
        
    def register_sample(self, sensor_id, index, value, timestamp):
        self.sensor_logs[sensor_id][index].set(value, timestamp)
    
    def get_sample(self, sensor_id, index):
        return self.sensor_logs[sensor_id][index]
        

# class DataHandler(IrSensor):
#     def __init__(self, num_sensors, max_samples):


# class IrSensor:
#     def __init__(self, mcp_channel: int):
#         self.mcp_channel = mcp_channel
        

# SENSOR_1 = 0
# SENSOR_2 = 1
# NUM_SENSORS = 2
# BUFFER_SIZE = 100


# data_stack = [[0] * BUFFER_SIZE] * NUM_SENSORS

# def read_sensor():
#     data_stack[SENSOR_1] = 



        
def main():
    current_readout_index = 0
    number_of_sensors = 2
    max_samples = 10
    
    sensors = [IrSensor(sensor_id) for sensor_id in range(number_of_sensors)]   # create list items to represent the sensors
    handler = SensorHandler(number_of_sensors, max_samples)
    
    while(True):
        for sensor_id, sensor in enumerate(sensors):
            handler.register_sample(sensor_id, current_readout_index, *sensor.get_sensor_data())
            #print(sensors[0].get_sensor_data())
    
        if(current_readout_index >= max_samples):
            current_readout_index = 0   # maximum buffer value reached, reset index to overwrite old data (FIFO)
            

    
    rows, cols = (5, 5)
    arr = [[0]*cols]*rows
    print(arr)
    
    arr[2][2] = 5
    print(arr)
    print(arr[2][1])
    
    
    # a = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8]]*2)
    # a = np.array([[3]*3]*2)
    # print(a[1, 2])
    # #a[2][2] = 3
    # print(a)
    # point = (1, 2)
    # print(a[point])
    
    # sensor_sample = SensorSample()
    # handler = SensorHandler(3, 4, sensor_sample)
    # print(type(handler.sensor_logs))
        
        
if __name__ == "__main__":
   main()