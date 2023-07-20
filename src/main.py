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
        #self.sensor_logs = np.array([[SensorSample()]*max_samples] * self.number_of_sensors)    # create number of objects needed to store the buffered sensor data
        self.sensor_logs = np.array([[SensorSample() for _ in range(max_samples)] for _ in range(self.number_of_sensors)])    # create number of objects needed to store the buffered sensor data
        
    def register_sample(self, sensor_id, index, value, timestamp):
        self.sensor_logs[sensor_id][index].set(value, timestamp)    
    
    def get_sample(self, sensor_id, index):
        return self.sensor_logs[sensor_id][index]
    
    def get_sensor_logs(self):
        return self.sensor_logs

        
def main():
    current_readout_index = 0
    number_of_sensors = 2
    max_samples = 10
    readout_frequency = 10  # in Hz
    
    sensors = np.array([IrSensor(sensor_id) for sensor_id in range(number_of_sensors)]) # create the rows in matrix to represent the sensors
    #print(sensors)
    handler = SensorHandler(number_of_sensors, max_samples)
    
    #while(True):
    for _ in range(15):
        for sensor_id, sensor in enumerate(sensors):
            #handler.register_sample(sensor_id, current_readout_index, *sensor.get_sensor_data())
            handler.register_sample(sensor_id, current_readout_index, *sensor.get_sensor_data())    # '*' unpacks the return tuple from function call
        
        print("_:", _)
        print("current_readout_index:", current_readout_index)    
        
        if(current_readout_index == 3):
            print("before overwritten buffer")
            print("sensor: 0; index: 3; value: {:d}; time: {:d}".format(handler.get_sample(0, 3).value, handler.get_sample(0, 3).timestamp))
            print("sensor: 1; index: 3; value: {:d}; time: {:d}".format(handler.get_sample(1, 3).value, handler.get_sample(1, 3).timestamp))

        current_readout_index += 1  # increase index to where store sensor readouts in the buffer
        
        if(current_readout_index >= max_samples):
            current_readout_index = 0   # when maximum buffer index is reached reset it to overwrite old data (FIFO)
            
        time.sleep(1/readout_frequency) # setting periodic time for sensor readout
        
    
    test = handler.get_sensor_logs()
    print(type(test))
    
    # print("after overwritten buffer")
    # print("sensor: 0; index: 3; value: {:d}; time: {:d}".format(handler.get_sample(0, 3).value, handler.get_sample(0, 3).timestamp))
    # print("sensor0_1", test[0][3].value, test[0][3].timestamp)
    # print("sensor: 1; index: 3; value: {:d}; time: {:d}".format(handler.get_sample(1, 3).value, handler.get_sample(1, 3).timestamp))
    # print("sensor1_1:", test[1][3].value, test[1][3].timestamp)
    
    print(test.shape)
    print(test)
        
if __name__ == "__main__":
   main()