import numpy as np
import torch
from torch import nn 
import os
import pandas as pd

# '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''#
#                                              Controller                                                    #
#                                                                                                            #
#""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""#
class PIDController:
    def __init__(self, kp, ki, kd, min_output, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.last_error = 0
        self.integral = 0

    def update(self, setpoint, feedback, dt):
        error = setpoint - feedback
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = min(self.max_output, max(self.min_output, output))
        self.last_error = error
        return output

class PositionController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.x_pid = PIDController(kp, ki, kd, 0)
        self.y_pid = PIDController(kp, ki, kd, 0)
        self.z_pid = PIDController(kp, ki, kd, 0)
        self.roll_pid = PIDController(kp, ki, kd, 0)
        self.pitch_pid = PIDController(kp, ki, kd, 0)
        self.yaw_pid = PIDController(kp, ki, kd, 0)

    def update(self, current_pose, desired_pose, dt):
        position_error = np.array(desired_pose[:3]) - np.array(current_pose[:3])
        orientation_error = np.array(desired_pose[3:]) - np.array(current_pose[3:])

        x_control = self.x_pid.update(position_error[0], dt)
        y_control = self.y_pid.update(position_error[1], dt)
        z_control = self.z_pid.update(position_error[2], dt)

        roll_control = self.roll_pid.update(orientation_error[0], dt)
        pitch_control = self.pitch_pid.update(orientation_error[1], dt)
        yaw_control = self.yaw_pid.update(orientation_error[2], dt)

        control = np.array([x_control, y_control, z_control, roll_control, pitch_control, yaw_control])
        return control

# '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''#
#                                              Filtering                                                    #
#                                                                                                            #
#""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""#

# source: https://www.samproell.io/posts/yarppg/yarppg-live-digital-filter/


class LiveFilter:
    """Base class for live filters.
    """
    def process(self, x):
        # do not process NaNs
        if np.isnan(x):
            return x

        return self._process(x)

    def __call__(self, x):
        return self.process(x)

    def _process(self, x):
        raise NotImplementedError("Derived class must implement _process")

from collections import deque

class LiveLFilter(LiveFilter):
    def __init__(self, b, a):
        """Initialize live filter based on difference equation.

        Args:
            b (array-like): numerator coefficients obtained from scipy.
            a (array-like): denominator coefficients obtained from scipy.
        """
        self.b = b
        self.a = a
        self._xs = deque([0] * len(b), maxlen=len(b))
        self._ys = deque([0] * (len(a) - 1), maxlen=len(a)-1)
    def _process(self, x):
        """Filter incoming data with standard difference equations.
        """
        self._xs.appendleft(x)
        y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
        y = y / self.a[0]
        self._ys.appendleft(y)

        return y


# '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''#
#                                              Predictor                                                    #
#                                                                                                            #
#""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""#

def predict(input,filename):
    
    num_hidden_units = 16
    model = CNNLSTM(num_sensors=6, hidden_units=num_hidden_units)
    
    output = torch.tensor([])
    
    path = "G:/My Drive/SoftExo/Repositories/trajectory_prediction_EMG/models/"
    path = os.path.join(path, filename)
    model.load_state_dict(torch.load(path))
    model.eval()
    
    input = torch.from_numpy(input).float()
    input = input.clone().detach()
  
  
    with torch.no_grad():
        y_star = model(input)
        output = torch.cat((output, y_star), 0)
    
    
    # input = np.reshape(input,(1,30,6))
        
    return y_star


def myPredictor(data_loader,name):
    """Just like `test_loop` function but keep track of the outputs instead of the loss
    function.
    """
    num_hidden_units = 16
    model = CNNLSTM(num_sensors=6, hidden_units=num_hidden_units)
    
    output = torch.tensor([])

    path = "G:/My Drive/SoftExo/Repositories/trajectory_prediction_EMG/models/"
    path = os.path.join(path, name)
    model.load_state_dict(torch.load(path))

    model.eval()
    with torch.no_grad():
        for X in data_loader:
            y_star = model(X)
            output = torch.cat((output, y_star), 0)
    
    return output


class CNNLSTM(nn.Module):
    def __init__(self, num_sensors, hidden_units):
        super().__init__()
        self.num_sensors = num_sensors  # this is the number of features
        self.hidden_units = hidden_units
        self.num_layers = 1

        self.lstm = nn.LSTM(
            input_size=num_sensors,
            hidden_size=hidden_units,
            batch_first=True,
            num_layers=self.num_layers
        )
        
        # input "image" size here is torch.Size([25, 1, 30, 6])
        self.cnn1 = nn.Conv2d(in_channels = 1, out_channels = 8, kernel_size = (5, 3), stride=(2, 1), padding=(4, 2))
        # height: input_size-filter_size +2(padding)/stride + 1 = 30-5+2(4)/2+1=17
        # width: 6-3+2*2/1 + 1 = 8
        # torch.Size([25, 8, 17, 8])
        self.batchnorm1 = nn.BatchNorm2d(8)
        # torch.Size([25, 8, 17, 8])
        # xcnnput_channel:8, batch(8)
        self.relu = nn.ReLU()
        # torch.Size([25, 8, 17, 8])
        self.maxpool1 = nn.MaxPool2d(kernel_size=(4,2))
        # torch.Size([25, 8, 4, 4])
        #input_size=28/2=14
        self.cnn2 = nn.Conv2d(in_channels=8, out_channels=32, kernel_size = (5, 1), stride=(2, 1), padding=(1, 1))
        # same_padding: (5-1)/2=2:padding_size. 
        # torch.Size([25, 32, 1, 6])
        self.batchnorm2 = nn.BatchNorm2d(32)
        self.maxpool2 = nn.MaxPool2d(kernel_size=1)
        self.fc1 = nn.Linear(in_features=192, out_features=96)
        # Nx3 * 3xO = NxO
        self.dropout = nn.Dropout(p=0.5)
        self.fc2 =nn.Linear(in_features=96, out_features=3)
        self.fc3 =nn.Linear(in_features=num_sensors, out_features=3)

        self.linear = nn.Linear(
            in_features=self.hidden_units, 
            out_features=3
            )

    def forward(self, x):
        
        batch_size = x.shape[0]
        
        # The CNN part:
        xcnn = x
        xcnn = xcnn.reshape(batch_size,1,x.shape[1],x.shape[2])
        xcnn = self.cnn1(xcnn)
        xcnn =self.batchnorm1(xcnn)
        xcnn =self.relu(xcnn)
        xcnn =self.maxpool1(xcnn)
        xcnn =self.cnn2(xcnn)
        xcnn =self.batchnorm2(xcnn)
        xcnn =self.relu(xcnn)
        xcnn =self.maxpool2(xcnn)
        xcnn = torch.flatten(xcnn,1)
        xcnn =self.fc1(xcnn)
        xcnn =self.relu(xcnn)
        xcnn =self.dropout(xcnn)
        xcnn =self.fc2(xcnn)
        
        # The LSTM part
        h0 = torch.zeros(self.num_layers, batch_size, self.hidden_units).requires_grad_()
        c0 = torch.zeros(self.num_layers, batch_size, self.hidden_units).requires_grad_()  # torch.Size([1, 25, 16])
        
        _, (hn, _) = self.lstm(x, (h0, c0))
        xlstm = self.linear(hn[0])  # First dim of Hn is num_layers, which is set to 3 above.

        
        # concat
        out = torch.cat((xcnn,xlstm), 1)
        out  = self.fc3(out)
        
        return out
    
    
    
    
# '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''#
#                                               DataLoaders                                                  #
#                                                                                                            #
#""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""#
import torch
from torch.utils.data import Dataset

class SequenceDataset(Dataset):
    def __init__(self, dataframe, target, features, sequence_length=5):
        self.features = features
        self.target = target
        self.sequence_length = sequence_length
        self.y = torch.tensor(dataframe[self.target].values).float()
        self.X = torch.tensor(dataframe[self.features].values).float()

    def __len__(self):
        return self.X.shape[0]

    def __getitem__(self, i): 
        if i >= self.sequence_length - 1:
            i_start = i - self.sequence_length + 1
            x = self.X[i_start:(i + 1), :]
        else:
            padding = self.X[0].repeat(self.sequence_length - i - 1, 1)
            x = self.X[0:(i + 1), :]
            x = torch.cat((padding, x), 0)

        return x, self.y[i]

class TestDataset(Dataset):
    def __init__(self, array, sequence_length=5):
        self.sequence_length = sequence_length
        self.X = torch.tensor(array).float()

    def __len__(self):
        return self.X.shape[0]

    def __getitem__(self, i): 
        if i >= self.sequence_length - 1:
            i_start = i - self.sequence_length + 1
            x = self.X[i_start:(i + 1), :]
        else:
            padding = self.X[0].repeat(self.sequence_length - i - 1, 1)
            x = self.X[0:(i + 1), :]
            x = torch.cat((padding, x), 0)

        return x
    
    
class PrepareDataset(torch.utils.data.Dataset):

    def __init__(self, file_name, window):
        prepare_df = pd.read_csv(file_name)
        
        self.window = window

        x = prepare_df.iloc[:, 0:6].values
        y = prepare_df.iloc[:, 3:6].values

        self.x_train = torch.tensor(x, dtype=torch.float32)
        self.y_train = torch.tensor(y, dtype=torch.float32)

    def __len__(self):
        return len(self.x_train) - self.window

    def __getitem__(self, idx):
        x = self.x_train[idx:idx+self.window]
        return x, self.y_train[idx]