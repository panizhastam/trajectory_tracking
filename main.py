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

pos_d = {'elbow': [0,0],
         'shfe': [0,0],
         'shaa': [0,0]}
SHAA_pwm = []
def ShAAcontrol(shaa_pos_control):


    if -0.1 < shaa_pos_control < 1.6:
        pos_d['shaa'][1] = shaa_pos_control
    else:
        pos_d['shaa'][1] = pos_d['shaa'][0]
    
    f = ((1.334)*pow(pos_d['shaa'][1],5)  +  (-5.584 )*pow(pos_d['shaa'][1],4) + (8.238)*pow(pos_d['shaa'][1],3) + (-4.944)*pow(pos_d['shaa'][1],2) + (0.3681)*pos_d['shaa'][1] + 2.8)*pos_d['shaa'][1]

    if f > 6:
        f = 6
    elif f < 0:
        f = 0
        

    ShAA_tao = (-384.08 * pow(((f*14.5038)/90), 4) + 789.52*pow(((f*14.5038)/90),3) - 618.81* pow(((f*14.5038)/90),2) + 468.18*((((f*14.5038))/90)))
    pos_d['shaa'][0] = pos_d['shaa'][1]
    return ShAA_tao
  
def ShFEcontrol(shfe_pos_control):


    if -0.1 < shfe_pos_control < 1.6:
        pos_d['shfe'][1] = shfe_pos_control
    else:
        pos_d['shfe'][1] = pos_d['shfe'][0]
    
    f = ((0.06433)*pow(pos_d['shfe'][1],3) + (-0.1573)*pow(pos_d['shfe'][1],2) + (-0.05318)*pos_d['shfe'][1] + 2.8)*pos_d['shfe'][1]
    
    if f > 6:
        f = 6
    elif f < 0:
        f = 0
        

    ShFE_tao = (-384.08 * pow(((f*14.5038)/90), 4) + 789.52*pow(((f*14.5038)/90),3) - 618.81* pow(((f*14.5038)/90),2) + 468.18*((((f*14.5038))/90)))
    pos_d['shfe'][0] = pos_d['shfe'][1]
    return ShFE_tao

def ELcontrol(el_pos_control):


    if -0.1 < el_pos_control < 1.6:
        pos_d['elbow'][1] = el_pos_control
    else:
        pos_d['elbow'][1] = pos_d['elbow'][0]
    
    f = ((0.0815)*pow(pos_d['elbow'][1],3) + (-0.2798)*pow(pos_d['elbow'][1],2) + (0.05135)*pos_d['elbow'][1] + 1.85)*(pos_d['elbow'][1])
    
    if f > 6:
        f = 6
    elif f < 0:
        f = 0
        

    Elbow_tao = (-384.08 * pow(((f*14.5038)/90), 4) + 789.52*pow(((f*14.5038)/90),3) - 618.81* pow(((f*14.5038)/90),2) + 468.18*((((f*14.5038))/90)))
    pos_d['elbow'][0] = pos_d['elbow'][1]
    return Elbow_tao


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

def runTest(td):
  numLoops = len(td)
  waitingForReply = False

  n = 0
  while n < numLoops:

    teststr = td[n]

    if waitingForReply == False:
      sendToArduino(teststr)
      print("Sent from PC -- LOOP NUM " + str(n) + " TEST STR " + teststr)
      waitingForReply = True

    if waitingForReply == True:

      while ser.inWaiting() == 0:
        pass
        
      dataRecvd = recvFromArduino()
      print("Reply Received  " + dataRecvd)
      n += 1
      waitingForReply = False

      print("===========")

    sleep(5)
    
    
    
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
from Functions import PIDController
from scipy.signal import butter

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


## My EMG data
mean_shaa_emg = 27.483195862361914
mean_shfe_emg = 25.14218503731391
mean_elbow_emg = 22.90115858854681
stdev_shaa_emg = 21.48593253986408
stdev_shfe_emg = 26.03970755397735
stdev_elbow_emg = 29.63859324352463

mean_shaa_pos = 0.20294872542728065
mean_shfe_pos = 0.1058734510546127
mean_elbow_pos = 0.36309130757248476
stdev_shaa_pos = 0.40153535433287646
stdev_shfe_pos = 0.2251798702190681
stdev_elbow_pos = 0.37357415426897334

mean_values = [mean_elbow_emg, mean_shfe_emg, mean_shaa_emg, mean_elbow_pos, mean_shfe_pos, mean_shaa_pos]
stdev_values = [stdev_elbow_emg, stdev_shfe_emg, stdev_shaa_emg, stdev_elbow_pos, stdev_shfe_pos, stdev_shaa_pos]


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


# Filtering variables
fs = 1/0.003
fc = 12  # cutoff frequency of the filter (Hz)
order = 3  # filter order
nyquist_freq = 0.5 * fs
Wn = fc / nyquist_freq  # cutoff frequency in the range [0, 1]
b, a = butter(order, Wn, btype='low')
live_bfilter = LiveLFilter(b, a)


startMarker = 60
endMarker = 62

iter = 0

waitForArduino()




## multiple batches        
myMatrix = np.zeros((30, 6))  
prep_data = [0,0,0,0,0,0] 
pwm = [0,0,0]  
filtered_emg = [0,0,0]
matrix_batch = []
while True:
  
  iter += 1
  
  dataRecvd = recvFromArduino()            
  data = [float(x) for x in dataRecvd.split(',')]
  angular_position = data[3:6]
  EMG_data = data[0:3]
            
            
  ## Filter the data
  for joint in np.arange(0,3):
      filtered_emg[joint] = live_bfilter(EMG_data[joint])
                
  filtered_data = np.concatenate((filtered_emg, angular_position), axis=None)
  
  
  ## Normalize the data
  prep_data = (filtered_data - np.array(mean_values)) / np.array(stdev_values)
              
  
  ## create sequential matrix
  myMatrix = np.roll(myMatrix, -1, axis=0)
  myMatrix[-1] = prep_data
  
  
  ## create number of batches
  # if len(matrix_batch)<25:
  #     matrix_batch.append(myMatrix.copy())
  #     continue
  
  matrix_array = np.array(myMatrix)       

  
  ## Feed to the LSTM-CNN model to predict the output
  matrix_array = np.reshape(matrix_array,(1,30,6))
  predicted_position = predict(matrix_array,"CNNLSTM_15ahead30past_12bw3_NoShuffle.pth").cpu().detach().numpy()
  
  
  ## rescale the output to the original amplitude
  predicted_position_rescaled = predicted_position * stdev_values[3:6] + mean_values[3:6]
  predicted_position_rescaled = predicted_position_rescaled.tolist()            
  
  ## clear the batch for the next one
  matrix_batch = []
  
  pwm = [ELcontrol(predicted_position_rescaled[-1][0]), ShFEcontrol(predicted_position_rescaled[-1][1]), ShAAcontrol(predicted_position_rescaled[-1][2])]
  
  command = "<{:.4f}".format(pwm[0])+","+"{:.4f}".format(pwm[1])+","+"{:.4f}>".format(pwm[2]) 
  print(command)

  send_command(command)
  
  
# while True:
#   iter += 1
#   dataRecvd = recvFromArduino()
#   # print(dataRecvd)

      
#   new_data = [float(x) for x in dataRecvd.split(',')]
#   sensors = new_data
#   angular_position = sensors[3:6]

#   # Add the new data to the end of the array
#   # The axis=0 argument specifies that we want to shift the rows (axis 0)
#   # and -1 specifies that we want to shift them up by one
#   if len(new_data) == 6: # Check if new_data has exactly 6 values
#       data = np.roll(data, -1, axis=0)
#       data[-1] = new_data

#   # predict 30 ms into the future
#   predicted_position = predict(data,"CNNLSTM3.pth").cpu().detach().numpy()
#   predicted_position = predicted_position.tolist()

#   # measure controller output
#   for joint in np.arange(0,3):
#       control_output[joint] = pid_controller.update(predicted_position[0][joint], angular_position[joint], dt)

#   # sending controller signal to arduino 
#   temp = "<{:.4f}".format(predicted_position[0][0])+","+"{:.4f}".format(predicted_position[0][1])+","+"{:.4f}>".format(predicted_position[0][2]) 
#   # print(temp)
#   if iter == 100:
#     ser.reset_input_buffer()
#     ser.reset_output_buffer()
#     iter = 0
#   send_command(temp)
#   print(temp)

# ser.close()
