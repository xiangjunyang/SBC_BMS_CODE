import os
import csv
import json
import serial
import time
import numpy as np
from datetime import datetime
from keras.models import load_model
from pykalman import KalmanFilter


class Data:
    def __init__(self):
        # print("init")
        # init class and load NN models
        self.SOC_Cha_model_files = "/home/orangepi/Desktop/project/SBC_BMS_CODE/models/SOC_Cha_cnn_lstm_model.h5"
        self.SOC_Discha_model_files = "/home/orangepi/Desktop/project/SBC_BMS_CODE/models/SOC_Discha_cnn_lstm_model.h5"
        self.Capacity_Cha_model_files = "/home/orangepi/Desktop/project/SBC_BMS_CODE/models/SOH_Cha_Capacity_model.h5"
        self.Capacity_Discha_model_files = "/home/orangepi/Desktop/project/SBC_BMS_CODE/models/SOH_Discha_Capacity_model.h5"
        self.SOH_Cha_model_files = "/home/orangepi/Desktop/project/SBC_BMS_CODE/models/SOH_Cha_model.h5"

        self.SOC_Cha_model = load_model(self.SOC_Cha_model_files)
        self.SOC_Discha_model = load_model(self.SOC_Discha_model_files)
        self.Capacity_Cha_model = load_model(self.Capacity_Cha_model_files)
        self.Capacity_Discha_model = load_model(
            self.Capacity_Discha_model_files)
        self.SOH_Cha_model = load_model(self.SOH_Cha_model_files)

        # init self register
        self.reg = [0 for i in range(27)]
        self.raw_writer = None
        self.basic_writer = None
        self.BMS_raw_file = None
        self.Basic_info = None
        self.SOH = 100
        self.SOC = 100
        self.V_avg_list = [0 for i in range(10)]
        self.V_avg_flag = 0
        self.response_json = None
        self.decode = None
        self.V_raw = 0
        self.Q_raw = 0
        self.I_raw = 0
        self.t_raw = 0
        self.V = 0
        self.Q = 0
        self.I = 0
        self.state = 0
        self.V_avg = 0
        self.Q_acc = 0
        self.Q_acc_old = 0

        print("load model successfully")

    @staticmethod
    def Inverse_1D(inputs, lower_limit, upper_limit):
        # to normalize input data for 1 dimension
        output = np.zeros(inputs.shape[0])
        for i in range(inputs.shape[0]):
            output[i] = abs(inputs[i] - lower_limit) / \
                abs(upper_limit - lower_limit)
        return output

    @staticmethod
    def Reverse_1D(inputs, lower_limit, upper_limit):
        # to convert output data to real data
        output = np.zeros(inputs.shape[0])
        for i in range(inputs.shape[0]):
            output[i] = ((inputs[i]) * abs(upper_limit -
                         lower_limit)) + lower_limit
        return output

    @staticmethod
    def Inverse_Single_input(inputs, lower_limit, upper_limit):
        output = abs(inputs - lower_limit) / abs(upper_limit - lower_limit)
        return output

    @staticmethod
    def Reverse_Single_input(inputs, lower_limit, upper_limit):
        output = (abs(upper_limit - lower_limit) * inputs) + lower_limit
        return output

    @staticmethod
    def Kalman1D(observations, damping=1):
        # To return the smoothed time series data
        observation_covariance = damping
        initial_value_guess = observations[0]
        transition_matrix = 1
        transition_covariance = 0.1
        initial_value_guess
        kf = KalmanFilter(
            initial_state_mean=initial_value_guess,
            initial_state_covariance=observation_covariance,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
            transition_matrices=transition_matrix,
        )
        pred_state, state_cov = kf.smooth(observations)
        return pred_state

    @staticmethod
    def sliding_avg(inputs, flag):
        # the method to calaulate average
        # output = []
        window_size = 10
        if (flag + 1) >= 10:
            output = sum(inputs) / window_size
        elif (flag + 1) > 0 and flag < 10:
            output = sum(inputs) / (flag + 1)
        return output

    @staticmethod
    def count_offset(V_SOH_list, Q_SOH_list):
        # the method to calaulate average
        V_offset = []
        Q_offset = []

        for i in range(0, len(V_SOH_list)):
            add_V_offset = V_SOH_list[i] - V_SOH_list[i - 1]
            add_Time_offset = 10
            add_mah_offset = Q_SOH_list[i] - Q_SOH_list[i - 1]
            V_offset.append(add_V_offset / add_Time_offset)
            Q_offset.append(add_mah_offset / add_V_offset)

        return V_offset, Q_offset

    def cal_SOH(self):
        self.V_SOH_list
        self.Q_SOH_list

        V_KF = Data.Kalman1D(self.V_SOH_list, 10)
        Q_KF = Data.Kalman1D(self.Q_SOH_list, 10)

        dVdT_arr, dQdV_arr = Data.count_offset(V_KF, Q_KF)

        dVdT_arr = np.array(dVdT_arr).astype(np.float)
        dQdV_arr = np.array(dQdV_arr).astype(np.float)

        VT_offset = Data.Inverse_1D(
            dVdT_arr, 0.00012779011509866933, 0.0002243848204071419
        )
        QV_offset = Data.Inverse_1D(
            dQdV_arr, 53084.241685698, 108073.87147044504)

        dVdT_arr = np.full((1, VT_offset.shape[0]), VT_offset)
        dQdV_arr = np.full((1, QV_offset.shape[0]), QV_offset)

        SOH_pred = self.SOH_Cha_model.predict((dVdT_arr, dQdV_arr))

        SOH = Data.Reverse_Single_input(
            SOH_pred[0][0], 94.42274119, 102.1402549)
        return SOH

    def CoulombCounter(self, I, SOC, Q, SOH):
        # CoulombCounter
        SOH = Data.Inverse_Single_input(SOH, 94.32827106, 102.1402549)  # SOH
        SOH_in = np.full((1, 1), SOH)  # SOH 0.98148148

        if I < 0:  # discharge
            Q_max = self.Capacity_Discha_model.predict(SOH_in)
            Q_max = Data.Reverse_Single_input(Q_max[0][0], 23844.14, 25818.84)
            SOC_CC = SOC + ((Q / Q_max) * 100)
        else:  # charge
            Q_max = self.Capacity_Cha_model.predict(SOH_in)
            Q_max = Data.Reverse_Single_input(Q_max[0][0], 23828.87, 25807.71)
            SOC_CC = SOC + ((Q / Q_max) * 100)

        return SOC_CC

    def cal_SOC(self, I, V, V_avg, SOH):
        # normalize
        if I < 0:  # discharge
            V = Data.Inverse_Single_input(V, 1.9854, 2.6913)  # V
            V_avg = Data.Inverse_Single_input(V_avg, 2.16588, 2.6913)  # V
            SOH = Data.Inverse_Single_input(SOH, 94.4227, 102.14025)  # SOH
        else:  # charge
            V = Data.Inverse_Single_input(V, 2.3042, 2.7021)  # V
            V_avg = Data.Inverse_Single_input(V_avg, 2.3042, 2.70029)  # V
            SOH = Data.Inverse_Single_input(SOH, 94.4227, 102.14025)  # SOH

        # input format
        V_in = np.full((1, 1), V)  # V 0.78415105
        V_avg_in = np.full((1, 1), V_avg)  # V 0.78415105
        SOH_in = np.full((1, 1), SOH)  # SOH 0.98148148

        # generate predict data format
        input_predict_data = V_in
        input_predict_data = np.append(input_predict_data, V_avg_in, 1)
        input_predict_data = np.append(input_predict_data, SOH_in, 1)

        # choose different models
        if I > 0:
            SOC_pred = self.SOC_Cha_model.predict(input_predict_data)
            print("charge mode")
        elif I < 0:
            SOC_pred = self.SOC_Discha_model.predict(input_predict_data)
            print("discharge mode")
        elif I == 0:
            SOC_pred = self.SOC_Discha_model.predict(input_predict_data)
            print("rest mode")
        SOC = Data.Reverse_Single_input(SOC_pred[0][0], 0, 100)
        print("forecast SOC = ", SOC)
        print("\n")
        return SOC

    def record_SOH_data(self, V, I, Q):
        # charge mode
        self.V_SOH_list = []
        self.Q_SOH_list = []
        if I > 0 and 2.65 >= V and V >= 2.45:
            self.V_SOH_list.append(V)
            self.Q_SOH_list.append(Q)
            if len(self.V_SOH_list) == 111:
                self.V_SOH_list = []
                self.Q_SOH_list = []
                return "cha_OK"
            else:
                return "cha_unready"

    def serial_setup(self, PORT):
        # pyserial init
        self.stmserial = serial.Serial()
        self.stmserial.port = PORT
        # 115200,N,8,1
        self.stmserial.baudrate = 57600
        self.stmserial.bytesize = serial.EIGHTBITS  # number of bits per bytes
        self.stmserial.parity = serial.PARITY_NONE  # set parity check
        self.stmserial.stopbits = serial.STOPBITS_ONE  # number of stop bits

        self.stmserial.timeout = 1  # non-block read 0.5s
        self.stmserial.writeTimeout = 0.8  # timeout for write 0.5s
        self.stmserial.xonxoff = False  # disable software flow control
        # disable hardware (RTS/CTS) flow control
        self.stmserial.rtscts = False
        # disable hardware (DSR/DTR) flow control
        self.stmserial.dsrdtr = False
        self.stmserial.open()

    def write_file(self):
        self.BMS_raw_file = open(self.Data_name01, "a", newline="")
        self.Basic_info = open(self.Data_name02, "a", newline="")
        self.raw_writer = csv.writer(self.BMS_raw_file)
        self.basic_writer = csv.writer(self.Basic_info)

    def open_file(self, Data_name01, Data_name02):
        self.Data_name01 = Data_name01
        self.Data_name02 = Data_name02
        if os.path.isfile(self.Data_name02):
            # If the file exists, catch old data to get the old state
            with open(self.Data_name02, "r") as f:
                for i, line in enumerate(f):
                    len = i + 1
                    data = line
                data = data.split(",")
                self.SOC = float(data[1])
                self.SOH = float(data[2])
            f.close()

            self.data_count_id = 1
            print("get self data count", self.data_count_id)

        else:
            # If the file doesn't exist, create it and write data to it
            self.BMS_raw_file = open(self.Data_name01, "w+", newline="")
            self.Basic_info = open(self.Data_name02, "w+", newline="")
            # write init data
            self.raw_writer = csv.writer(self.BMS_raw_file)
            self.raw_writer.writerow(
                [
                    "Data_id",
                    "BMS_Num",
                    "state",
                    "error code",
                    "Stack Voltage(mV)",
                    "cell current(mA)",
                    " tempture(*C)",
                    "SOC",
                    "SOH",
                    "Current_Time",
                    "cell_voltage1",
                    "cell_voltage2",
                    "cell_voltage3",
                    "cell_voltage4",
                    "cell_voltage5",
                    "cell_voltage6",
                    "cell_voltage7",
                    "cell_voltage8",
                    "cell_voltage9",
                    "cell_voltage10",
                    "cell_voltage11",
                    "cell_voltage12",
                    "cell_voltage13",
                    "cell_voltage14",
                    "cell_voltage15",
                    "cell_voltage16",
                    "cell_voltage17",
                    "cell_voltage18",
                    "cell_voltage19",
                    "cell_voltage20",
                    "Acc_Charge",
                    "Pass_Q",
                ]
            )

            self.basic_writer = csv.writer(self.Basic_info)
            self.basic_writer.writerow(["V", "SOC", "SOH"])
            self.BMS_raw_file.close()
            self.Basic_info.close()
            self.data_count_id = 1
            print("get self data count", self.data_count_id)

    @staticmethod
    def decode_err_code(err_code):
        if err_code == 0:
            return "COMMUNICATION_ERROR"
        elif err_code == 1:
            return "OVER_TEMP_ALARM"
        elif err_code == 3:
            return "UNDER_TEMP_ALARM"
        elif err_code == 5:
            return "OVER_CHARGE_CURRENT_ALARM"
        elif err_code == 7:
            return "OVER_DISCHARGE_CURRENT_ALARM"
        elif err_code == 9:
            return "OVER_VOLT_ALARM"
        elif err_code == 11:
            return "UNDER_VOLT_ALARM"
        elif err_code == 17:
            return "VOLTAGE_IMBALANCE_WARNING"

    def read_record_raw_data(self):
        self.V_avg_list = [0 for i in range(10)]
        self.V_avg_flag = 0

        for i in range(10):
            # read BMS raw data for 1 second
            self.response_json = self.stmserial.readline()
            try:
                self.decode = json.loads(self.response_json)
                self.V_raw = self.decode["battery_voltage"]
                self.Q_raw = self.decode["Pass_Q"]
                self.I_raw = self.decode["BMS1_pack_current"]
                self.t_raw = [self.decode["BMS1_TS1Temp"],
                              self.decode["BMS1_TS3Temp"]]
            except json.decoder.JSONDecodeError as e:
                print(f"Encountered Error: {e}")

            # decode V I data and set init data as input
            self.V = self.V_raw / (20 * 1000)
            self.Q = self.Q_raw
            self.I = self.I_raw / 1000
            self.Q_acc += self.Q

            # set 0 second init V_avg SOC
            if self.V_avg_flag == 0:
                self.V_avg = self.V
                SOC_NN = Data.cal_SOC(
                    self, self.I, self.V, self.V_avg, self.SOH)
                self.SOC = SOC_NN

            # t = temperature,sort from high to low,use the higher as temp
            t = sorted(self.t_raw, reverse=True)
            self.Temp = t[0] / 100

            # decode battery state
            if self.decode["battery_state"] == "OK":
                self.state = 0
                error_state = " "
                error_code = 9999
            else:
                self.state = 1
                error_state = Data.decode_err_code(
                    self.decode["BMS_error_code"])
                error_code = self.decode["BMS_error_code"]

            self.reg = [
                self.state,
                error_code,
                self.decode["battery_voltage"],
                self.decode["BMS1_pack_current"],
                self.Temp,
                self.SOC,
                self.SOH,
                self.decode["cell_voltage0"],
                self.decode["cell_voltage1"],
                self.decode["cell_voltage2"],
                self.decode["cell_voltage3"],
                self.decode["cell_voltage4"],
                self.decode["cell_voltage5"],
                self.decode["cell_voltage6"],
                self.decode["cell_voltage7"],
                self.decode["cell_voltage8"],
                self.decode["cell_voltage9"],
                self.decode["cell_voltage10"],
                self.decode["cell_voltage11"],
                self.decode["cell_voltage12"],
                self.decode["cell_voltage13"],
                self.decode["cell_voltage14"],
                self.decode["cell_voltage15"],
                self.decode["cell_voltage16"],
                self.decode["cell_voltage17"],
                self.decode["cell_voltage18"],
                self.decode["cell_voltage19"],
            ]

            if i == 9:
                # set pass Q filter
                if self.data_count_id == 1:
                    self.Q_acc_old = self.Q_acc
                if (abs(self.Q_acc) - abs(self.Q_acc_old)) > 100:
                    self.Q_acc = self.Q_acc_old
                else:
                    self.Q_acc_old = self.Q_acc

                # cal SOC and record SOH data every 10 seconds
                self.V_avg_list[self.V_avg_flag] = self.V
                self.V_avg = Data.sliding_avg(self.V_avg_list, self.V_avg_flag)
                self.V_avg_flag += 1
                if self.SOC > 90:
                    SOC_NN = Data.cal_SOC(
                        self, self.I, self.V, self.V_avg, self.SOH)
                    SOC_CC = Data.CoulombCounter(
                        self, self.I, self.SOC, self.Q_acc, self.SOH
                    )
                    self.SOC = (SOC_NN * 0.1) + (SOC_CC * 0.9)
                else:
                    SOC_NN = Data.cal_SOC(
                        self, self.I, self.V, self.V_avg, self.SOH)
                    SOC_CC = Data.CoulombCounter(
                        self, self.I, self.SOC, self.Q_acc, self.SOH
                    )
                    self.SOC = (SOC_NN * 0.6) + (SOC_CC * 0.4)
                print("id = ", self.data_count_id)
                now_time = datetime.now()

                # SOH calculation
                if self.V > 2.65 and self.I > 20:
                    self.SOH = 100.2522
                # SOH_state = Data.record_SOH_data(self, self.V, self.I, self.Q)
                # if SOH_state == "cha_OK":
                #     self.SOH = Data.cal_SOH(self)
                #     pass
                # else:
                #     pass

                print("V=", self.decode["battery_voltage"])
                print("I=", self.decode["BMS1_pack_current"])
                print("Pass Q=", self.Q_acc)
                print("Acc_Charge=", self.decode["BMS1_AccumulatedCharge"])
                print("Temp=", self.Temp)
                print("SOC=", self.SOC)
                print("SOH=", self.SOH)
                print("\n")

                self.write_file()

                # record basic battery information every 10 seconds
                self.raw_writer.writerow(
                    [
                        self.data_count_id,
                        "BMS_1",
                        self.state,
                        error_state,
                        self.decode["battery_voltage"],
                        self.decode["BMS1_pack_current"],
                        self.Temp,
                        self.SOC,
                        self.SOH,
                        now_time,
                        self.decode["cell_voltage0"],
                        self.decode["cell_voltage1"],
                        self.decode["cell_voltage2"],
                        self.decode["cell_voltage3"],
                        self.decode["cell_voltage4"],
                        self.decode["cell_voltage5"],
                        self.decode["cell_voltage6"],
                        self.decode["cell_voltage7"],
                        self.decode["cell_voltage8"],
                        self.decode["cell_voltage9"],
                        self.decode["cell_voltage10"],
                        self.decode["cell_voltage11"],
                        self.decode["cell_voltage12"],
                        self.decode["cell_voltage13"],
                        self.decode["cell_voltage14"],
                        self.decode["cell_voltage15"],
                        self.decode["cell_voltage16"],
                        self.decode["cell_voltage17"],
                        self.decode["cell_voltage18"],
                        self.decode["cell_voltage19"],
                        self.Q_acc,
                        self.decode["BMS1_AccumulatedCharge"],
                    ]
                )
                self.basic_writer.writerow([self.V, self.SOC, self.SOH])
                self.data_count_id += 1

                # update register data every 10 seconds
                self.reg = [
                    self.state,
                    error_code,
                    self.decode["battery_voltage"],
                    self.decode["BMS1_pack_current"],
                    self.Temp,
                    self.SOC,
                    self.SOH,
                    self.decode["cell_voltage0"],
                    self.decode["cell_voltage1"],
                    self.decode["cell_voltage2"],
                    self.decode["cell_voltage3"],
                    self.decode["cell_voltage4"],
                    self.decode["cell_voltage5"],
                    self.decode["cell_voltage6"],
                    self.decode["cell_voltage7"],
                    self.decode["cell_voltage8"],
                    self.decode["cell_voltage9"],
                    self.decode["cell_voltage10"],
                    self.decode["cell_voltage11"],
                    self.decode["cell_voltage12"],
                    self.decode["cell_voltage13"],
                    self.decode["cell_voltage14"],
                    self.decode["cell_voltage15"],
                    self.decode["cell_voltage16"],
                    self.decode["cell_voltage17"],
                    self.decode["cell_voltage18"],
                    self.decode["cell_voltage19"],
                ]
                self.BMS_raw_file.close()
                self.Basic_info.close()
                self.Q_acc = 0

        time.sleep(0.1)


def Get_new_data(port, data_name1, data_name2):
    NEW_DATA = Data()
    NEW_DATA.serial_setup(port)
    NEW_DATA.open_file(data_name1, data_name2)
    # NEW_DATA.read_record_raw_data()
    while 1:
        NEW_DATA.read_record_raw_data()


if __name__ == "__main__":
    Get_new_data(
        "/dev/ttyUSB0",
        "./output_data/BMS1_Output_raw_data.csv",
        "./output_data/BMS1_Basic_data.csv",
    )
