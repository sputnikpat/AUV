import os     #importing os library so as to communicate with the system
import time   #importing time library to make Rpi wait because its too impatient 
os.system ("sudo pigpiod") #Launching GPIO library
time.sleep(1) # As i said it is too impatient and so if this delay is removed you will get an error
import pigpio #importing GPIO library

ESC=4  #Connect the ESC in this GPIO pin
ESCright=6
ESCleft=17

pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC, 0)
pi.set_servo_pulsewidth(ESCright, 0)
pi.set_servo_pulsewidth(ESCleft, 0)

max_value = 2000 #change this if your ESC's max value is different or leave it be
min_value = 700  #change this if your ESC's min value is different or leave it be


def calibrate():   #This is the auto calibration procedure of a normal ESC
    pi.set_servo_pulsewidth(ESC, 0)
    pi.set_servo_pulsewidth(ESCright, 0)
    pi.set_servo_pulsewidth(ESCleft, 0)
    print("Disconnect the battery and press Enter")
    inp = input()
    if inp == '':
        pi.set_servo_pulsewidth(ESC, max_value)
        pi.set_servo_pulsewidth(ESCright, max_value)
        pi.set_servo_pulsewidth(ESCleft, max_value)
        print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = input()
        if inp == '':            
            pi.set_servo_pulsewidth(ESC, min_value)
            pi.set_servo_pulsewidth(ESCright, min_value)
            pi.set_servo_pulsewidth(ESCleft, min_value)
            print ("Wierd eh! Special tone")
            time.sleep(7)
            print ("Wait for it ....")
            time.sleep (5)
            print ("Im working on it, DONT WORRY JUST WAIT.....")
            pi.set_servo_pulsewidth(ESC, 0)
            pi.set_servo_pulsewidth(ESCright, 0)
            pi.set_servo_pulsewidth(ESCleft, 0)
            time.sleep(2)
            print ("Arming ESC now...")
            arm()
            # time.sleep(1)
            # print ("See.... uhhhhh")
            #control() # You can change this to any other function you want
                  
def arm(): #This is the arming procedure of an ESC 
    print ("Press Enter")
    inp = input()    
    if inp == '':
        pi.set_servo_pulsewidth(ESC, 0)
        pi.set_servo_pulsewidth(ESCright, 0)
        pi.set_servo_pulsewidth(ESCleft, 0)
        time.sleep(1)
        pi.set_servo_pulsewidth(ESC, max_value)
        pi.set_servo_pulsewidth(ESCright, max_value)
        pi.set_servo_pulsewidth(ESCleft, max_value)
        time.sleep(1)
        pi.set_servo_pulsewidth(ESC, min_value)
        pi.set_servo_pulsewidth(ESCright, min_value)
        pi.set_servo_pulsewidth(ESCleft, min_value)
        time.sleep(1)
        speed = 1500 
        pi.set_servo_pulsewidth(ESC, speed)
        pi.set_servo_pulsewidth(ESCright, speed)
        pi.set_servo_pulsewidth(ESCleft, speed)
        while (speed>=1200):
            speed-=100
        pi.set_servo_pulsewidth(ESC, speed)
        pi.set_servo_pulsewidth(ESCright, speed)
        pi.set_servo_pulsewidth(ESCleft, speed)
        while(speed<2000):
            speed+=100
            pi.set_servo_pulsewidth(ESC, speed)
            pi.set_servo_pulsewidth(ESCright, speed)
            pi.set_servo_pulsewidth(ESCleft, speed)
            time.sleep(2)
            control() 
             
def control(): 
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
               
def neeche(): #going down
    speed=2000
    pi.set_servo_pulsewidth(ESC,speed)
    pi.set_servo_pulsewidth(ESCright,min_value)
    pi.set_servo_pulsewidth(ESCleft,min_value) 
    
def left(): #going left
    
    speed=2000
    pi.set_servo_pulsewidth(ESC,speed)
    pi.set_servo_pulsewidth(ESCright,speed)
    pi.set_servo_pulsewidth(ESCleft,min_value) 
    #time.sleep(15)
    
def right(): #going right
    
    speed=2000
    pi.set_servo_pulsewidth(ESC,speed)
    pi.set_servo_pulsewidth(ESCright,min_value)
    pi.set_servo_pulsewidth(ESCleft,speed)
    #time.sleep(15)
    
def forward(): #going forward
    
    speed=2000
    pi.set_servo_pulsewidth(ESC,speed)
    pi.set_servo_pulsewidth(ESCright,speed)
    pi.set_servo_pulsewidth(ESCleft,speed)
    #time.sleep(15)
    
def stop(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC, 0)
    pi.set_servo_pulsewidth(ESCright, 0)
    pi.set_servo_pulsewidth(ESCleft, 0)
    pi.stop()
                    


#This is the start of the program actually, to start the function it needs to be initialized before calling... stupid python.    
inp = input()
if inp == "manual":
    manual_drive()
elif inp == "calibrate":
    calibrate()
elif inp == "arm":
    arm()
elif inp == "control":
    control()
elif inp == "stop":
    stop()
else :
    print ("Thank You for not following the things I'm saying... now you gotta restart the program STUPID!!")


        
