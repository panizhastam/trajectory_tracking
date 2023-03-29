import serial
import numpy as np
from time import sleep
import csv
from Functions import *

ser = serial.Serial('COM5',
                    baudrate= 115200,
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


##############################################################################################################
######                                         main loop                                                  ####
##############################################################################################################

with open('train_data3.csv', 'a',newline='') as f:
    iter = 0
    while True:        
        try:
            iter += 1
            # read the sensor data from the serial connection
            serial_data = ser.read_until(b'\n').decode('iso-8859-1').strip()
            
            # Convert the string to a list of floats
            new_data = [float(x) for x in serial_data.split(',')]
            sensors = new_data
            angular_position = sensors[3:6]
            
 
            if len(sensors) == 6:
                writer = csv.writer(f)
                writer.writerow(sensors)
            
            
            
            # print(sensors)
            
            # Add the new data to the end of the array
            # The axis=0 argument specifies that we want to shift the rows (axis 0)
            # and -1 specifies that we want to shift them up by one
            # if len(new_data) == 6: # Check if new_data has exactly 6 values
            #     data = np.roll(data, -1, axis=0)
            #     data[-1] = new_data
            
            # # predict 30 ms into the future
            # predicted_position = predict(data).cpu().detach().numpy()
            # predicted_position = predicted_position.tolist()
            
            # # measure controller output
            # for joint in np.arange(0,3):
            #     control_output[joint] = pid_controller.update(predicted_position[0][joint], angular_position[joint], dt)

            # # sending controller signal to arduino
            # control_output_str = str(control_output)  
            # temp = "<{:.4f}".format(control_output[0])+","+"{:.4f}".format(control_output[1])+",""{:.4f}>".format(control_output[2]) 
            # print(temp)
            
            # ser.write(temp)
                
            # flushing the buffer after 100 iterations to accelerate process        
            if iter == 100:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                iter = 0

        except: 
            continue
        
        
        
        



