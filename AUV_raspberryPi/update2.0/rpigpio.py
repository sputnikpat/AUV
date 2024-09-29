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

    def control(self):
        print("I'm Starting the motor, I hope its calibrated and armed, if not restart by giving 'x'")
        sleep(1)
        speed = 1500 # change your speed if you want to.... it should be between 700 - 2000
        print("Speed Controls - a -->decrease & d -->increase OR q -->decrease more & e --> increase more")
        while True:
            self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, speed)
            inp = input()
            if inp == "q":
                speed -= 100 # decrementing the speed like hell
                print("speed = %d" % speed)
            elif inp == "e":
                speed += 100 # incrementing the speed like hell
                print("speed = %d" % speed)
            elif inp == "d":
                speed += 10 # incrementing the speed
                print("speed = %d" % speed)
            elif inp == "a":
                speed -= 10 # decrementing the speed
                print("speed = %d" % speed)
            elif inp == "stop":
                self.stop() # going for the stop function
                break
            elif inp == "manual":
                self.manual_drive()
                break
            elif inp == "arm":
                self.arm()
                break
            else:
                print("WHAT DID I SAID!! Press a,q,d or e")

    def arm(self): # This is the arming procedure of an ESC
        #print("Connect the battery and press Enter")
        print("Starting arming with assumption as battery is connected to ESC")
        #inp = input()
        #if inp == '':
        self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, 0)
        sleep(1)
        self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, PWM_MAX_VAL)
        sleep(1)
        self.pi_pwm.set_servo_pulsewidth(self.gpio_pin, PWM_MIN_VAL)
        sleep(1)
        # self.control()

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
