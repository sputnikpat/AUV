#########################################################################
# File: main_rpi_control.py 
# Purpose: Main RPi controlling file.
# Owner: SS
# Note: This program is going to control RPi for PWM operations on GPIO Pins.
#########################################################################

from esc_data import ESC_DATA
from config import *
import os

try:
    import rpigpio
except Exception as e:
    print("Failed to import GPIO module...")
 
import server_control
import socket
import time
import re


class common():
    
    def __init__(self):
        try:
             # start pigpiod demon if not started
            if USE_PIGPIO_MODULE:
                print("Start pigpiod demon...")
                os.system("export PIGPIO_ADDR=soft, export PIGPIO_PORT=8888")
                time.sleep(2)
                os.system("sudo pigpiod")
                time.sleep(3)
        except Exception as e:
            print("Either demon is already running or failed to start: {}".format(e))
        self.client_conns = None
    
    @staticmethod
    def create_socket (host,  tcp_port ):
        server_socket = None
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((host, tcp_port))
            server_socket.listen(1)
        except Exception as e:
            print("Failed to create socket: {}".format(e))
        return server_socket
     
  
    def get_server_socks(self):
        servers = {}
        # Read ESC data from config
        for esc, data in ESC_DATA.items():
            port = data.get('PORT')
            if port:
                server_sock = self.create_socket(LOCALHOST, port)
                if server_sock:
                    servers[esc] = server_sock
            else:
                print("No PORT configured in ESC_DATA for ESC: {}".format(esc))
        return servers
        
        
    def perform_setup(self, server_socks, client_conns):
        for esc, s in server_socks.items():
            if not client_conns.get(esc) :
                client_conns[esc] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_conns[esc] .connect((LOCALHOST, ESC_DATA.get(esc).get('PORT')))
                clientsock, clientAddress = s.accept()
                client_conns[esc] .sendall(bytes("1500", 'UTF-8'))
                # print("Connection accepted....")
                newthread = server_control.ClientThread(clientAddress, clientsock, ESC_DATA.get(esc).get('PORT'), ESC_DATA.get(esc).get('GPIO'))
                print("Thread created: {}".format(newthread))
                newthread.start()
                server_client_threads.append(newthread)
        print("Threads are created for receiving commands for GPIO PWM")
        print(client_conns, server_client_threads)
        self.client_conns = client_conns
        return (client_conns, server_client_threads)
            
    @staticmethod
    def get_cmd_data(cmd):
        data = re.findall(r'esc\d|[0-9]+', cmd)
        esc_val = {}
        if data:
            esc = None
            for i in data:
                if 'esc' in i:
                    esc = i
                    esc_val[esc] = 0
                else:
                    if esc and i.isdigit():
                        esc_val[esc] = i
        return esc_val
    
    def run_demo(self):
        # Method to run all ESC in esc_data file under demo mode.
        i = 1
        count = 0
        keep_run = True
        demo_start_vals = {}#holds speed
        demo_stop_stats = {}#if stop
        demo_rpigpio_obj = {}#to call class RPIGPIO 
        
        for esc in self.client_conns:
            demostart = ESC_DATA.get(esc).get('DEMO')
            if demostart:
                demo_start_vals[esc] = demostart
                demo_stop_stats[esc] = False
                gpio_pin = ESC_DATA.get(esc).get('GPIO')
                demo_rpigpio_obj[esc] = rpigpio.HandleGpioPgpio(gpio_pin, frequency=PWM_FREQUENCY, pwm_duration=0)
                print("Arming ESC; {}".format(esc))
                demo_rpigpio_obj[esc].arm()
                
        print("***** Khili khuli hai .. Apni demo life ..... *****")
        while keep_run:
            count += i
            if count >100:
                i = 100
            elif count > 10:
                i = 10
            else:
                i = 1
            for esc in demo_start_vals: 
                print("sending value {} to gpio pin# {} of esc: {}".format(demo_start_vals[esc], ESC_DATA.get(esc).get('GPIO'), esc))
                self.client_conns[esc].sendall(bytes(str(demo_start_vals[esc]) ,'UTF-8'))
                demo_start_vals[esc]  += i
                if demo_start_vals[esc] >= PWM_MAX_VAL:
                    demo_stop_stats[esc] = True
                    print("Reached max PWM for {}".format(esc))
                keep_run = not(all(demo_stop_stats.values()))
            time.sleep(2)
        print("All ESCs are at their max value of PWM... Lets stop after 30 Secs... Sleeping ;)")
        time.sleep(30)
        for esc in demo_start_vals:
            demo_rpigpio_obj[esc].stop()
            self.client_conns[esc].sendall(bytes('bye', 'UTF-8'))
            self.client_conns[esc].close()
        print("Killing pigpiod demon...")
        os.system("sudo killall pigpiod")
        time.sleep(3)
        print("***** Puri band hui .. Apni demo life ..... ***** ")
        return
        
    
    def process_run_cmd(self, cmd):
        # Check whats in command to run..
        esc_val = {}
        if "calibrate" in cmd or "arm"  in cmd or "control" in cmd:
            esc = re.findall(r'esc\d', cmd)
            #create pgpio object for esc to calibrate
            gpio_pin = ESC_DATA.get(esc[0].upper()).get('GPIO')
            esc_pgpio_obj = rpigpio.HandleGpioPgpio(gpio_pin, frequency=PWM_FREQUENCY, pwm_duration=0)
            if  "calibrate" in cmd:
                esc_pgpio_obj.calibrate()
            elif "control" in cmd:
                esc_pgpio_obj.control()
            elif "arm" in cmd:
                esc_pgpio_obj.arm()
            else:
                pass
        elif "demo" in cmd:
                self.run_demo()
        else:
                esc_val = self.get_cmd_data(cmd)
        return esc_val
        
    @staticmethod
    def add_new_esc(new_esc):
        for key in new_esc:
            if key not in ESC_DATA:
                ESC_DATA.update(new_esc)
                break
        # save new ESC data for future use
        
        try:
            fo = open('esc_data.py', 'w')
            fo.write("ESC_DATA = "+str(ESC_DATA))
            fo.close()
        except Exception as e:
            print("Failed to update new ESC data....:{}".format(e))
        
# Main 
if __name__ == '__main__':

    index = 0
    server_socks = {}
    server_client_threads = []
    client_conns = {}
    common_methods = common()
    
    # Check if it has to run in Demo mode
    
    if DEMO_MODE_ON:
        print("********* Yuuuhuuuu starting Apani demo life with setup first *********")
        server_socks = common_methods. get_server_socks()
        (client_conns, server_client_threads) = common_methods.perform_setup(server_socks, client_conns)
        time.sleep(3)
        print("Lets Rock it....")
        common_methods.run_demo()            
    else:
        # Read command from user
        while True:
            print("******* User input ********")
            print("Please select option: s --> Setup; r -->Run Commands;  a--> Add new ESC;  bye --> End the test")
            operation  = input().lower()
            
            if operation != 's' and not server_socks:
                print ("Setup not ready... ... Proceeding with Setup first..") 
                server_socks = common_methods. get_server_socks()
                (client_conns, server_client_threads) = common_methods.perform_setup(server_socks, client_conns)
                time.sleep(3)
                
            if operation== 's':
                print("Setting it up.....")
                server_socks = common_methods. get_server_socks()
                (client_conns, server_client_threads) = common_methods.perform_setup(server_socks, client_conns)
                time.sleep(3)
                
            elif operation == 'r':
                print("Please enter command... Whatever U wants to do man.... with GPIO..:)")
                cmd = input().lower()
                esc_val = common_methods.process_run_cmd(cmd)
                if esc_val:
                    for esc, value in esc_val.items():
                        client_conns[esc.upper()].sendall(bytes(value,'UTF-8'))
                else:
                    print("No valid esc number or pwm values in commands....")
                    
            elif operation == 'a':
                new_esc = {}
                print("Adding new ESC: Please enter: ESC#, <GPIO pin#> <port> ")
                esc_data = input()
                r = re.findall(r'ESC\d|[0-9]+|[0-9]+', esc_data.upper())
                if len(r) >=3 and "ESC" in r[0]:
                    if r[2].isdigit():
                        port = int(r[2])
                    if r[1].isdigit():
                        gpio = int(r[1])
                    new_esc[r[0]] = {'PORT':port, 'GPIO':gpio}
                    common_methods.add_new_esc(new_esc)
                    # Need to restart the test after addition of new esc 
                    print("Need to restart the test ... killing all threads..... Bye")
                    for client in client_conns.values():
                        client.sendall(bytes('bye', 'UTF-8'))
                    break
                else:
                    print("Invalid data please try again")
                
            elif operation == 'bye':
                print("Ending the test and killing all threads..... Bye")
                for client in client_conns.values():
                    client.sendall(bytes('bye', 'UTF-8'))
                    client.close()
                print("Killing pigpiod demon...")
                os.system("sudo killall pigpiod")
                time.sleep(3)
                break
            else: 
                print("Invalid input please try again:    Enter option: s --. Setup; r -->Run Commands;  a--> Add new ESC")
                
            time.sleep(1)
             
    """    
    if server_client_threads:
        for th in server_client_threads:
            print("starting Thread: {}".format(th))
            th.start()
            time.sleep(2)
    """
