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

# ADC converter of ir sensor analog voltage signal to digital input
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))


class IrSensor:
    def __init__(self, mcp_channel: int):
        self.mcp_channel = mcp_channel
        self.value = 0
        self.timestamp = 0
        self.database = []
    
    def read_sensor_value(self):
        self.value = mcp.read_adc(self.mcp_channel)
        self.timestamp = round(time.time()*1000)    # current time in milliseconds
        return self.value, self.timestamp


class DataHandler(IrSensor):
    def __init__(self, max_samples):
        self.max_samples = max_samples
        self.sensor1 = IrSensor(0)
        self.sensor2 = IrSensor(1)
        self.database_sensor1 = []
        self.database_sensor2 = []
    
    def store_data(self):
        # read sensor data and store to local member variable
        self.sensor1.read_sensor_value()
        self.sensor1.read_sensor_value()
    
        self.database_sensor1.append(self.sensor1.timestamp)
        self.database_sensor1.append(self.sensor1.value)
        self.database_sensor2.append(self.sensor2.timestamp)
        self.database_sensor2.append(self.sensor2.value)
        
        # only store last max_samples number of sensor readouts
        if(len(self.database_sensor1) > self.max_samples):
            for _ in range(2):
                self.database_sensor1.pop(0)
                self.database_sensor2.pop(0)
            
        
    def get_database(self, sensor_select:int):
        if(sensor_select == 1):
            return self.database_sensor1
        elif(sensor_select == 2):
            return self.database_sensor2
        

# create DataHandler obj
datahandler = DataHandler(10)

for i in range(30):
    datahandler.store_data()
    time.sleep(0.1)

print(datahandler.get_database(1))
