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
        self.sensor_logs = [[SensorSample()]*max_samples] * self.number_of_sensors    # create number of objects needed to store the buffered sensor data
        
    def register_sample(self, sensor_id, index, value, timestamp):
        # ???????? SET FUNKTIONEN NEDAN KOMMER INTE UPP SOM FÖRSLAG NÄR MAN SKRIVER .set SÅ TROR INTE KOPPLINGEN TILL SensorSample OBJEKTET ÄR RÄTT
        self.sensor_logs[sensor_id][index].set(value, timestamp)    
    
    def get_sample(self, sensor_id, index):
        return self.sensor_logs[sensor_id][index]
    
    def get_sensor_logs(self):
        return self.sensor_logs


        
def main():
    current_readout_index = 0
    number_of_sensors = 2
    max_samples = 10
    
    sensors = [IrSensor(sensor_id) for sensor_id in range(number_of_sensors)]   # create list items to represent the sensors
    handler = SensorHandler(number_of_sensors, max_samples)
    
    #while(True):
    for _ in range(15):
        for sensor_id, sensor in enumerate(sensors):    # ?????? SKAPAR DENNA FUNKTION VERKLIGEN TVå OLIKA SENSORER (OLIKA SENSOR ID) OCH POPULERAR RESPEKTIVE RAD I SENSOR_LOGS MATRISEN?
            handler.register_sample(sensor_id, current_readout_index, *sensor.get_sensor_data())
            
        if(current_readout_index >= max_samples):
            current_readout_index = 0   # when maximum buffer value is reached reset index to overwrite old data (FIFO)
        
        print("sensor1; value: {:d}, timestamp: {:d}".format(handler.get_sample(0, 2).value, handler.get_sample(0, 2).timestamp))
        print("sensor2; value: {:d}, timestamp: {:d}".format(handler.get_sample(1, 2).value, handler.get_sample(1, 2).timestamp))
        # sensor1_id = 0
        # sensor2_id = 1
        # for i in range(max_samples):
        #     print("[{:d}][{:d}]: {:d}; {:d}".format(sensor1_id, i, handler.get_sample(sensor1_id, i).value, handler.get_sample(sensor1_id, i).timestamp))
        #     print("[{:d}][{:d}]: {:d}; {:d}".format(sensor2_id, i, handler.get_sample(sensor2_id, i).value, handler.get_sample(sensor2_id, i).timestamp))
        time.sleep(0.1)

    # ??????? NÄR JAG PRINTAR sensor_logs FÖR DE TVÅ RADERNA SOM REPRESENTERAR sensor1 RESP. sensor2 SÅ SKRIVER DEN UT SAMMA VÄRDE. VERKAR SOM BARA LÄSTA VÄRDET FÖR ENA SENSORS POPULERAR BÅDA RADERNA I sample_logs
    test = handler.get_sensor_logs()
    print("sensor1", test[0][1].value)
    print("sensor2", test[1][1].value)
    #print("test; value: {:d}, timestamp: {:d}".format(test.value, test.timestamp))
        
    
    
    
    # rows, cols = (5, 5)
    # arr = [[0]*cols]*rows
    # print(arr)
    
    # arr[2][2] = 5
    # print(arr)
    # print(arr[2][1])
    
    
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