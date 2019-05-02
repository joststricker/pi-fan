import os
import RPi.GPIO as GPIO
import time
import datetime


# Time to sleep between checking the temperature
sleepTime = 30

# want to write unbuffered to file
fileLog = open('/home/pi/pi-fan.log', 'w+', 0)


def time_stamp():
    """Log messages should be time stamped"""
    t = time.time()
    s = datetime.datetime.fromtimestamp(t).strftime('%Y/%m/%d %H:%M:%S - ')
    return s


def print_msg(s):
    """Write messages in a standard format"""
    fileLog.write(time_stamp() + s + "\n")
    # print(time_stamp() + s + "\n")


class Fan: 
    off = True
    speed = 0
    pin = 0
    fan = None
    
    def __init__(self, gpio_pin, fan_speed):
        self.speed = fan_speed
        self.pin = gpio_pin
        self.fan = None
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        self.fan = GPIO.PWM(self.pin, 25)

        print_msg("Initialized: fan attached to GPIO pin: " + str(self.pin))

    """Start the fan"""
    def start_fan(self):
        print_msg("Starting fan")
        self.off = False
        GPIO.output(self.pin, GPIO.HIGH)

        print_msg(str(self.fan))
        self.fan.start(float(self.speed))

    """Stop the fan"""
    def stop_fan(self):
        print_msg("Stopping fan")
        self.off = True
        GPIO.output(self.pin, GPIO.LOW)

    """Make the fan spin faster"""
    def increase_speed(self):

        self.speed = self.speed + 10
        if self.speed > 100:
            self.speed = 100
        
        print_msg("Increasing speed to: " + str(float(self.speed)))

        self.fan.ChangeDutyCycle(float(self.speed))

    """Make the fan spin slower"""
    def decrease_speed(self):
        self.speed = self.speed - 10
        if self.speed < 20:
            self.speed = 20
        
        print_msg("Decreasing speed to: " + str(float(self.speed)))
        
        self.fan.ChangeDutyCycle(float(self.speed))


class CPUTemp:
    temperature = 0.0
    previoustemperature = 0.0

    def __init__(self):
        self.get_temperature()

    def get_temperature(self):
        self.previoustemperature = self.temperature

        res = os.popen('/opt/vc/bin/vcgencmd measure_temp').readline()
        self.temperature = float((res.replace("temp=", "").replace("'C\n", "")))
        return self.temperature

    def test_temp(self, cputemp):
        self.previoustemperature = cputemp
        return cputemp


class FanController:
    gpio_pin = 0
    fan_speed = 0
    max_cpu_temp = 0
    fan = None
    cputemperature = None

    def __init__(self, gpiopin, speed, maxtemp):
        self.gpio_pin = gpiopin
        self.fan_speed = speed
        self.max_cpu_temp = maxtemp
        self.cputemperature = CPUTemp()
        self.fan = Fan(self.gpio_pin, self.fan_speed)

    """Monitor the cpu temp and control fan accordingly"""
    def monitor(self):
        try:
            while True:
                # see above
                cpu_temp = self.cputemperature.get_temperature()
                print_msg("CPU temp: " + str(cpu_temp))

                if cpu_temp > self.max_cpu_temp:
                    """Start fan"""
                    if self.fan.off is True:
                        self.fan.start_fan()
                        """Sleep should be added here somewhere"""
                    else:
                        if self.cputemperature.temperature >= self.cputemperature.previoustemperature:
                            self.fan.increase_speed()
                        elif self.cputemperature.temperature <= self.cputemperature.previoustemperature:
                            self.fan.decrease_speed()
                    time.sleep(10)

                else:
                    if self.fan.off is False:
                        self.fan.stop_fan()
                        
                    # Sleep for quite some time
                    time.sleep(20)

        except KeyboardInterrupt:
            # GPIO.output(18, GPIO.LOW)
            """Exit program on CTRL+C"""


fanController = FanController(18, 50, 60.0)
fanController.monitor()
