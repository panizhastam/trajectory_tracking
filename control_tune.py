import pandas as pd
import numpy as np
import csv
from Functions import *
import ast
from scipy.signal import butter, filtfilt





# df = pd.read_csv('control_data.csv')

# Controller variables
kp = 2
ki = 0.05
kd = 0.1
min_output = 0
max_output = 6
pid_controller = PIDController(kp, ki, kd, min_output, max_output)
dt = 0.003
control_output = [0, 0, 0]

# Filtering variables
fs = 1/0.003
fc = 12  # cutoff frequency of the filter (Hz)
order = 3  # filter order
nyquist_freq = 0.5 * fs
Wn = fc / nyquist_freq  # cutoff frequency in the range [0, 1]
b, a = butter(order, Wn, btype='low')
live_bfilter = LiveLFilter(b, a)

filtered_emg = [0,0,0]
matrix_batch = []

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


pos_d = [0,0]
SHAA_pwm = []

def ShAAcontrol(shaa_pos_control):


    if -0.1 < shaa_pos_control < 1.6:
        pos_d[1] = shaa_pos_control
    else:
        pos_d[1] = pos_d[0]
    
    f = ((1.334)*pow(pos_d[1],5)  +  (-5.584 )*pow(pos_d[1],4) + (8.238)*pow(pos_d[1],3) + (-4.944)*pow(pos_d[1],2) + (0.3681)*pos_d[1] + 2.8)*pos_d[1]

    if f > 6:
        f = 6
    elif f < 0:
        f = 0
        

    ShAA_tao = (-384.08 * pow(((f*14.5038)/90), 4) + 789.52*pow(((f*14.5038)/90),3) - 618.81* pow(((f*14.5038)/90),2) + 468.18*((((f*14.5038))/90)))
    pos_d[0] = pos_d[1]
    return ShAA_tao





## multiple batches        
myMatrix = np.zeros((30, 6))  
prep_data = [0,0,0,0,0,0] 
SHAA_pwm = []     
with open('control_data2.csv', 'a',newline='') as f:

    with open('train_data.csv', 'r') as csvfile:
        csvreader = csv.reader(csvfile)

        # Iterate over the rows in the CSV file
        for i, row in enumerate(csvreader):
            
            data = [float(x) for x in row]
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
            
            
            # ## create number of batches
            # if len(matrix_batch)<25:
            #     matrix_batch.append(myMatrix.copy())
            #     continue
            
            # matrix_array = np.array(matrix_batch)       

            
            ## Feed to the LSTM-CNN model to predict the output
            matrix_array = np.reshape(myMatrix,(1,30,6))
            predicted_position = predict(matrix_array,"CNNLSTM_15ahead30past_12bw3_NoShuffle.pth").cpu().detach().numpy()
            
            
            ## rescale the output to the original amplitude
            predicted_position_rescaled = predicted_position * stdev_values[3:6] + mean_values[3:6]
            predicted_position_rescaled = predicted_position_rescaled.tolist()            
            
            ## clear the batch for the next one
            matrix_batch = []
            
            
            ## applies PID control
            for joint in np.arange(0,3):
                control_output[joint] = pid_controller.update(predicted_position_rescaled[-1][joint], angular_position[joint], dt)

            
            writer = csv.writer(f)
            writer.writerow(control_output)
            
            # ## apply gravity compensation and convert to pwm
            # for row in predicted_position_rescaled:
            #     SHAA_pwm.append(ShAAcontrol(row[2]))
            
            
            # writer = csv.writer(f,lineterminator='\n')                            
            # for i,rows in enumerate(SHAA_pwm):    
            #     writer.writerow([rows]) 
                
            # SHAA_pwm = []






# ## Running train data on the trained model to see the results
# ## Single batch 
# myMatrix = np.zeros((30, 6))
# with open('control_data2.csv', 'a',newline='') as f:

#     with open('train_data.csv', 'r') as csvfile:
#         csvreader = csv.reader(csvfile)

#         # Iterate over the rows in the CSV file
#         for i, row in enumerate(csvreader):
            
#             data = [float(x) for x in row]
#             angular_position = data[3:6]
#             EMG_data = data[0:3]
            
#             for joint in np.arange(0,3):
#                 filtered_emg[joint] = live_bfilter(EMG_data[joint])
                
#             preprocessed_data = np.concatenate((filtered_emg, angular_position), axis=None)

            
#             myMatrix = np.roll(myMatrix, -1, axis=0)
#             myMatrix[-1] = preprocessed_data            
            
    
#             pos = np.reshape(myMatrix,(1,30,6))   
              
#             predicted_position = predict(pos,"CNNLSTM_15ahead30past_12bw3_NoShuffle.pth").cpu().detach().numpy()
#             predicted_position = predicted_position.tolist()
                        
            
#             # values = ast.literal_eval(predicted_position[0])
#             # print(predicted_position)
            
#             writer = csv.writer(f)
#             writer.writerow(predicted_position[0])


# ## using filtering after 25 batches
# from torch.utils.data import DataLoader
# torch.manual_seed(101)

# batch_size = 25
# sequence_length = 30
# prev_pos = [0.0,0.0,0.0]
# new_pred = [0.0,0.0,0.0]

# myMatrix = np.zeros((sequence_length, 6))
# with open('control_data2.csv', 'a',newline='') as f:

#     with open('train_data.csv', 'r') as csvfile:
#         csvreader = csv.reader(csvfile)

#         # Iterate over the rows in the CSV file
#         for i, row in enumerate(csvreader):
            
#             data = [float(x) for x in row]
#             angular_position = data[3:6]
#             EMG_data = data[0:3]
            
#             for joint in np.arange(0,3):
#                 filtered_emg[joint] = live_bfilter(EMG_data[joint])
                
#             preprocessed_data = np.concatenate((filtered_emg, angular_position), axis=None)
           
#             myMatrix = np.roll(myMatrix, -1, axis=0)
#             myMatrix[-1] = preprocessed_data
            
#             matrix_batch.append(myMatrix.copy())

#             if len(matrix_batch) < 25:
#                 continue
            
            
#             matrix_array = np.array(matrix_batch)
            
#             # test_dataset = TestDataset(
#             #     matrix_array,
#             #     sequence_length = sequence_length
#             # )
            
#             # train_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)
             
            
   
#             predicted_position = predict(matrix_array,"CNNLSTM_15ahead30past_12bw3_batch25.pth").cpu().detach().numpy()
#             predicted_position = predicted_position.tolist()
                        
#             matrix_batch = []
            
#             for i,rows in enumerate(predicted_position):
#                 for joint in np.arange(0,3):
                    
#                     if abs(predicted_position[i][joint] - prev_pos[joint]) > 0.1:
#                         new_pred[joint] = prev_pos[joint]
#                     else: 
#                         new_pred[joint] = predicted_position[i][joint]
                        
#                 writer = csv.writer(f,lineterminator='\n')
#                 writer.writerow(new_pred) 
#                 prev_pos = new_pred
            

                


# # Open the input and output CSV files
# with open('output.csv', 'r') as input_file, open('output2.csv', 'w', newline='') as output_file:
#     csv_reader = csv.reader(input_file)
#     csv_writer = csv.writer(output_file)

#     # Get the column index that you want to adjust
#     col_index = 4  # Change this to the appropriate column index

#     # Initialize the offset and the previous value
#     offset = 0
#     prev_value = 0

#     # Iterate over the rows of the input CSV file
#     for row in csv_reader:
#         # Get the value in the column
#         value = float(row[col_index])

#         # Check if the value is negative
#         if value < prev_value and value <=0:
#             # Update the offset and the previous value
#             offset = value

        

#         # Adjust the value using the offset
#         value -= offset

#         # Check if the value is less than the previous negative value
        
              

#         # Check if the value is positive again
#         if value< 0 and prev_value >= 0:
#             # Reset the previous value and the offset if the value is positive
#             offset = 0
        
#         prev_value = value
        
#         row[col_index] = value 
#         # Write the row to the output CSV file
#         csv_writer.writerow(row)



# with open('output.csv', 'r') as input_file, open('control_data3.csv', 'w', newline='') as output_file:

#     csv_reader = csv.reader(input_file)
#     csv_writer = csv.writer(output_file)
    
#     for row in csv_reader:
        
#         data = [float(x) for x in row]
#         angular_position = [data[3:6]]

#         # predict 30 ms into the future
#         predicted_position = predict(angular_position).cpu().detach().numpy()
#         predicted_position = predicted_position.tolist()

#         for joint in np.arange(0,3):
#             control_output[joint] = pid_controller.update(predicted_position[0][joint], angular_position[joint], dt)
            
#         print(control_output)

