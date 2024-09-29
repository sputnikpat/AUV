import RPi.GPIO as GPIO
from time import sleep
import pigpio # importing GPIO library
from config import *

class HandleGpio():
    
    def __init__(self, gpio_pin, frequency, pwm_duration = 0):
        self.gpio_pin = gpio_pin
        self.pwm_duration = pwm_duration / 1000
        
        self.pi_pwm = None
        self.max_duration = (1.0 / frequency) * 1000
        self.pwmdc = (self.pwm_duration / self.max_duration) *100
        
        GPIO.setwarnings(False)			#disable warnings
        GPIO.setmode(GPIO.BOARD)		#set pin numbering system
        GPIO.setup(self.gpio_pin, GPIO.OUT)
        self.pi_pwm = GPIO.PWM(self.gpio_pin, frequency)		#create PWM instance with frequency
        self.pi_pwm.start(self.pwmdc)				#start PWM of required Duty Cycle 
        
    def set_pwm_dur(self, pwm_duration):
        self.pwm_duration = pwm_duration / 1000
        self.pwmdc = (self.pwm_duration / self.max_duration)*100
        print("Set DC: {} % on GPIO: {}".format(self.pwmdc, self.gpio_pin))
        self.pi_pwm.ChangeDutyCycle(self.pwmdc) #provide duty cycle in the range 0-100
        sleep(0.01)
        
    def __del__(self):
        # Kill all pwm objs.
        try:
            if self.pi_pwm:
                self.pi_pwm.stop()
        except Exception as e:
            print("Fail to stop PWM objs: {}".format(e))
            
            

# Child class using parent class above for handling pgpio
class HandleGpioPgpio(HandleGpio):

    def __init__(self, gpio_pin, frequency, pwm_duration=0):
        super().__init__(gpio_pin, frequency, pwm_duration=pwm_duration)
        self.gpio_pin = gpio_pin
        self.pwm_duration = self.pwm_duration
        
        self.pi_pwm = pigpio.pi()
        self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, self.pwm_duration)
        self.pwm_obj_stopped = False

    def set_pwm_dur(self, pwm_duration):
        self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, pwm_duration)
        sleep(0.01)

    def get_pwm_dur(self):
        pwm_duration = self.pi_pwm.get_servo_pulsewidth(self.gpio_pin)
        return pwm_duration

    def manual_drive(self): # You will use this function to program your ESC if required
        print("You have selected manual option so give a value between 0 and you max value")
        while True:
            inp = input()
            if inp == "stop":
                self.stop()
                break
            elif inp == "control":
                self.control()
                break
            elif inp == "arm":
                self.arm()
                break
            else:
                self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, inp)
                
    escDown1 = self.pi_pwm.gpio_pin.get(ESC1)
    escDown2 = self.pi_pwm.gpio_pin.get(ESC2)
    escLeft = self.pi_pwm.gpio_pin.get(ESC3)
    escRight = self.pi_pwm.gpio_pin.get(ESC4)
    
    def neeche(): #going down
        speed=2000
        self.pi_pwm.set_servo_pulsewidth(escDown1,speed)
        self.pi_pwm.set_servo_pulsewidth(escDown2,speed)
        self.pi_pwm.set_servo_pulsewidth(escRight,700)
        self.pi_pwm.set_servo_pulsewidth(escLeft,700) 
    
    def left(): #going left
        
        speed=2000
        self.pi_pwm.set_servo_pulsewidth(escDown1,speed)
        self.pi_pwm.set_servo_pulsewidth(escDown2,speed)
        self.pi_pwm.set_servo_pulsewidth(escRight,speed)
        self.pi_pwm.set_servo_pulsewidth(escLeft,700) 
        #time.sleep(15)
        
    def right(): #going right
        
        speed=2000
        self.pi_pwm.set_servo_pulsewidth(escDown1,speed)
        self.pi_pwm.set_servo_pulsewidth(escDown2,speed)
        self.pi_pwm.set_servo_pulsewidth(escRight,700)
        self.pi_pwm.set_servo_pulsewidth(escLeft,speed) 
        #time.sleep(15)
        
    def forward(): #going forward
        
        speed=2000
        self.pi_pwm.set_servo_pulsewidth(escDown1,speed)
        self.pi_pwm.set_servo_pulsewidth(escDown2,speed)
        self.pi_pwm.set_servo_pulsewidth(escRight,speed)
        self.pi_pwm.set_servo_pulsewidth(escLeft,speed) 
        #time.sleep(15)


    def control(self):
        while True:
            inp = input("Enter down, forward, left or right: ")
            if inp == "down":
                neeche()
            elif inp == "left":    
                left()
            elif inp == "forward":    
                forward()
            elif inp == "right":
                right()
            elif inp == "stop":
                stop()          #going for the stop function
                break
        
    def arm(self): # This is the arming procedure of an ESC
        print ("Connect the battery and press Enter")
        inp = input()    
        if inp == '':
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, 0)
            # pi.set_servo_pulsewidth(ESCright, 0)
            # pi.set_servo_pulsewidth(ESCleft, 0)
            time.sleep(1)
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, max_value)
            # pi.set_servo_pulsewidth(ESCright, max_value)
            # pi.set_servo_pulsewidth(ESCleft, max_value)
            time.sleep(1)
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, min_value)
            # pi.set_servo_pulsewidth(ESCright, min_value)
            # pi.set_servo_pulsewidth(ESCleft, min_value)
            time.sleep(1)
            speed = 1500 
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, speed)
            # pi.set_servo_pulsewidth(ESCright, speed)
            # pi.set_servo_pulsewidth(ESCleft, speed)
            while (speed>=1200):
                speed-=100
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, speed)
            # pi.set_servo_pulsewidth(ESCright, speed)
            # pi.set_servo_pulsewidth(ESCleft, speed)
            while(speed<2000):
                speed+=100
                self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, speed)
                # pi.set_servo_pulsewidth(ESCright, speed)
                # pi.set_servo_pulsewidth(ESCleft, speed)
                time.sleep(2)
                control()

    def stop(self): # This will stop every action your Pi is performing for ESC ofcourse.
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, 0)
            self.pi_pwm.stop()
            self.pwm_obj_stopped = True
            print("PWM Obj stopped for gpio Pin: {}".format(self.gpio_pin))

    def calibrate(self): # This is the auto calibration procedure of a normal ESC
        self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, 0)
        print("Disconnect the battery and press Enter")
        inp = input()
        if inp == '':
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, PWM_MAX_VAL)
            print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press "
            "Enter")
            inp = input()
        if inp == '':
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, PWM_MIN_VAL)
            print("Wierd eh! Special tone")
            sleep(7)
            print("Wait for it ....")
            sleep(5)
            print("Im working on it, DONT WORRY JUST WAIT.....")
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, 0)
            sleep(2)
            print("Arming ESC now...")
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, PWM_MIN_VAL)
            sleep(1)
            print("See.... uhhhhh")
            self.control() # You can change this to any other function you want

    def __del__(self):
        # Kill all pwm objs.
        try:
            if not self.pwm_obj_stopped:
                # print("I see PWM obj is running...{}".format(self.pi_pwm))
                self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, 0)
                self.pi_pwm.stop()
        except Exception as e:
            print("Fail to stop PWM objs: {}".format(e))
