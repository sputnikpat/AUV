#########################################################################
# File: server_control.py 
# Purpose: contains class to Create multiple threads for sockets.
# Owner: SS
# Note: This program is going to get integrated on RPi for PWM operations on GPIO Pins.
#########################################################################
import time
import socket, threading
from config import *

try:
    import rpigpio
except Exception as e:
    print("Failed to import GPIO module...")
 
#########################################################################
# Class for creating Threads for each request from client
#########################################################################
class ClientThread(threading.Thread):

    def __init__(self, clientAddress, clientsocket, port, gpio):
        threading.Thread.__init__(self)
        self.csocket = clientsocket
        self.clientAddress = clientAddress
        self.port = port
        self.gpio = gpio
         
        #create instance of rpi GPIO class
        try:
            if USE_PIGPIO_MODULE:
                self.gpioHandle = rpigpio.HandleGpioPgpio(self.gpio, frequency=PWM_FREQUENCY, pwm_duration = 0)
            else:
                self.gpioHandle = rpigpio.HandleGpio(self.gpio, frequency=PWM_FREQUENCY, pwm_duration = 0)
        except Exception as e:
            print("Fail to start PWM handler...{}".format(e))
             
        print ("New connection added: {} : {} for GPIO {} oprations ".format(clientAddress, port, gpio))

        
    def run(self):
        count_empty_msg = 0
        print ("Connection from :  {}".format(self.clientAddress))
        #self.csocket.send(bytes("Hi, This is from Server..",'utf-8'))
        msg = ''
        while True:
            data = self.csocket.recv(2048)
            msg = data.decode()
            if msg=='bye':
                break
            else:
                if msg == "":
                    count_empty_msg += 1
                    if count_empty_msg >= MAX_EMPTY_MSG_COUNT:
                        print("Looks like client is disconnected... ending server connection")
                        break
                    time.sleep(2)
                else:
                    print("Command received from client on port : {} for GPIO {} : msg / PWM value :{}".format(self.port, self.gpio, msg))
                    # Process the command based on expected operations on the GPIO pin.
                    if msg.isdigit():
                        # Send the pulse duration to gpio pin
                        try:
                            self.gpioHandle.set_pwm_dur(int(msg))
                        except Exception as e:
                            print("Failed to send GPIO PWM value")
            if msg:    
                print ("From Server: ", msg)
                self.csocket.send(bytes(msg,'UTF-8'))
            
        print ("Client at ", self.clientAddress , " : ", self.port,  " disconnected...")

# Main 
if __name__ == '__main__':

    pass
