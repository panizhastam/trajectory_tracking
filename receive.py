# 19 July 2014

# in case any of this upsets Python purists it has been converted from an equivalent JRuby program

# this is designed to work with ... ArduinoPC2.ino ...

# the purpose of this program and the associated Arduino program is to demonstrate a system for sending 
#   and receiving data between a PC and an Arduino.

# The key functions are:
#    sendToArduino(str) which sends the given string to the Arduino. The string may 
#                       contain characters with any of the values 0 to 255
#
#    recvFromArduino()  which returns an array. 
#                         The first element contains the number of bytes that the Arduino said it included in
#                             message. This can be used to check that the full message was received.
#                         The second element contains the message as a string


# the overall process followed by the demo program is as follows
#   open the serial connection to the Arduino - which causes the Arduino to reset
#   wait for a message from the Arduino to give it time to reset
#   loop through a series of test messages
#      send a message and display it on the PC screen
#      wait for a reply and display it on the PC

# to facilitate debugging the Arduino code this program interprets any message from the Arduino
#    with the message length set to 0 as a debug message which is displayed on the PC screen

# the message to be sent to the Arduino starts with < and ends with >
#    the message content comprises a string, an integer and a float
#    the numbers are sent as their ascii equivalents
#    for example <LED1,200,0.2>
#    this means set the flash interval for LED1 to 200 millisecs
#      and move the servo to 20% of its range

# receiving a message from the Arduino involves
#    waiting until the startMarker is detected
#    saving all subsequent bytes until the end marker is detected

# NOTES
#       this program does not include any timeouts to deal with delays in communication
#
#       for simplicity the program does NOT search for the comm port - the user must modify the
#         code to include the correct reference.
#         search for the lines 
#               serPort = "/dev/ttyS80"
#               baudRate = 9600
#               ser = serial.Serial(serPort, baudRate)
#


#=====================================

#  Function Definitions

#=====================================

def sendToArduino(sendStr):
  ser.write(sendStr.encode())


#======================================

def recvFromArduino():
  global startMarker, endMarker
  
  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many
  
  # wait for the start character
  while  ord(x) != startMarker: 
    x = ser.read()
  
  # save data until the end marker is found
  while ord(x) != endMarker:
    if ord(x) != startMarker:
      ck = ck + x.decode()
      byteCount += 1
    x = ser.read()
  
  return(ck)


#============================

def waitForArduino():

   # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
   # it also ensures that any bytes left over from a previous message are discarded
   
    global startMarker, endMarker
    
    msg = ""
    while msg.find("Arduino is ready") == -1:

      while ser.inWaiting() == 0:
        pass
        
      msg = recvFromArduino()

      print(msg)

      
#======================================

    
    
    
def send_command(teststr):

    waitingForReply = False

    if waitingForReply == False:
        sendToArduino(teststr)
        # print("Sent from PC"  + " TEST STR " + teststr)
        waitingForReply = True

    if waitingForReply == True:

        while ser.inWaiting() == 0:
            pass


#======================================

# THE PROGRAM STARTS HERE

#======================================

import serial
import numpy as np
from time import sleep
import csv
from Functions import *

serPort = 'COM5'
baudRate = 115200
ser = serial.Serial(port=serPort,
                    baudrate= baudRate,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1,
                    xonxoff=0,
                    rtscts=0)  # replace 'COM3' with the appropriate port for your Arduino
# Toggle DTR to reset Arduino
ser.setDTR(False)
sleep(2)
# toss any data already received, see
# http://pyserial.sourceforge.net/pyserial_api.html#serial.Serial.flushInput
ser.flushInput()
ser.setDTR(True)


data = np.zeros((30, 6))


# Controller variables
kp = 0.5
ki = 0.05
kd = 0.1
min_output = -1.0
max_output = 1.0
pid_controller = PIDController(kp, ki, kd, min_output, max_output)
dt = 0.002
control_output = [0, 0, 0]
print("Serial port " + serPort + " opened  Baudrate " + str(baudRate))


startMarker = 60
endMarker = 62

iter = 0

waitForArduino()
with open('control_data2.csv', 'a',newline='') as f:
    while True:
        iter += 1
        dataRecvd = recvFromArduino()
        # print(dataRecvd)

            
        new_data = [float(x) for x in dataRecvd.split(',')]
        sensors = new_data


        if len(sensors) == 6:
            writer = csv.writer(f)
            writer.writerow(sensors)
                    
        send_command("<>")
    
ser.close()

