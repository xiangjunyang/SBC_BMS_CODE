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
        self.SOC_Cha_model_files = "./models/SOC_Cha_FNN_model.h5"
        self.SOC_Discha_model_files = "./models/SOC_Discha_FNN_model.h5"
        self.SOH_Cha_model_files = "./models/SOH_Cha_model.h5"

        self.SOC_Cha_model = load_model(self.SOC_Cha_model_files)
        self.SOC_Discha_model = load_model(self.SOC_Discha_model_files)
        self.SOH_Cha_model = load_model(self.SOH_Cha_model_files)

        # SOH only use Charge to estimate
        # self.SOH_Discha_model_files = './models/SOH_Discha_model.h5'
        # self.SOH_Discha_model = load_model(self.SOH_Discha_model_files)

        # init self register
        self.reg = [
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]
        print("load model successfully")

    def Inverse_1D(inputs, lower_limit, upper_limit):
        # to normalize input data for 1 dimension
        output = np.zeros(inputs.shape[0])
        for i in range(inputs.shape[0]):
            output[i] = abs(inputs[i] - lower_limit) / abs(upper_limit - lower_limit)
        return output

    def Reverse_1D(inputs, lower_limit, upper_limit):
        # to convert output data to real data
        output = np.zeros(inputs.shape[0])
        for i in range(inputs.shape[0]):
            output[i] = ((inputs[i]) * abs(upper_limit - lower_limit)) + lower_limit
        return output

    def Inverse_Single_input(inputs, lower_limit, upper_limit):
        output = abs(inputs - lower_limit) / abs(upper_limit - lower_limit)
        return output

    def Reverse_Single_input(inputs, lower_limit, upper_limit):
        output = (abs(upper_limit - lower_limit) * inputs) + lower_limit
        return output

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

    def sliding_avg(inputs, flag):
        # the method to calaulate average
        output = []
        window_size = 10
        if flag >= 10:
            output = sum(inputs) / window_size
        elif flag > 0 and flag < 10:
            output = sum(inputs[0:flag]) / flag
        return output

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
        QV_offset = Data.Inverse_1D(dQdV_arr, 53084.241685698, 108073.87147044504)

        dVdT_arr = np.full((1, VT_offset.shape[0]), VT_offset)
        dQdV_arr = np.full((1, QV_offset.shape[0]), QV_offset)

        SOH_pred = self.SOH_Cha_model.predict((dVdT_arr, dQdV_arr))

        SOH = Data.Reverse_Single_input(SOH_pred[0][0], 94.42274119, 102.1402549)
        return SOH

    def cal_SOC(self, I, V, V_avg, SOH):
        # # normalize
        # if I < 0:  # discharge
        #     V = Data.Inverse_Single_input(V, 1.9854, 2.6913)  # V
        #     V_avg = Data.Inverse_Single_input(V, 2.16588, 2.6913)  # V
        #     SOH = Data.Inverse_Single_input(SOH, 94.4227, 102.14025)  # SOH
        # else:  # charge
        #     V = Data.Inverse_Single_input(V, 2.3042, 2.7021)  # V
        #     V_avg = Data.Inverse_Single_input(V, 2.3042, 2.70029)  # V
        #     SOH = Data.Inverse_Single_input(SOH, 94.4227, 102.14025)  # SOH

        I = -50
        V = 0.98951693
        V_avg = 0.98951693
        SOH = 0.64019746

        print("cal SOC V", V)
        print("cal SOC V_avg", V_avg)
        print("cal SOC SOH", SOH)

        # input format
        V_in = np.full((1, 1), V)  # V 0.78415105
        V_avg_in = np.full((1, 1), V_avg)  # V 0.78415105
        SOH_in = np.full((1, 1), SOH)  # SOH 0.98148148

        # choose different models
        if I > 0:
            SOC_pred = self.SOC_Cha_model.predict((V_in, V_avg_in, SOH_in))
            print("charge mode")
        elif I < 0:
            SOC_pred = self.SOC_Discha_model.predict((V_in, V_avg_in, SOH_in))
            print("discharge mode")
        elif I == 0:
            SOC_pred = self.SOC_Discha_model.predict((V_in, V_avg_in, SOH_in))
            print("rest mode")
        SOC = Data.Reverse_Single_input(SOC_pred[0][0], 0, 100)
        print("forecast SOC = ", SOC)
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
        self.stmserial.baudrate = 576000
        self.stmserial.bytesize = serial.EIGHTBITS  # number of bits per bytes
        self.stmserial.parity = serial.PARITY_NONE  # set parity check
        self.stmserial.stopbits = serial.STOPBITS_ONE  # number of stop bits

        self.stmserial.timeout = 1.0  # non-block read 0.5s
        self.stmserial.writeTimeout = 0.5  # timeout for write 0.5s
        self.stmserial.xonxoff = False  # disable software flow control
        # disable hardware (RTS/CTS) flow control
        self.stmserial.rtscts = False
        # disable hardware (DSR/DTR) flow control
        self.stmserial.dsrdtr = False
        self.stmserial.open()

    def open_file(self, Data_name01, Data_name02):
        self.Data_name01 = Data_name01
        self.Data_name02 = Data_name02

        if os.path.isfile(Data_name02):
            # If the file exists, catch old data to get the old state
            with open(Data_name02, "r") as f:
                for i, line in enumerate(f):
                    len = i + 1
                    data = line
                data = data.split(",")
                self.SOC = float(data[1])
                self.SOH = float(data[2])

            self.BMS_raw_file = open(self.Data_name01, "a", newline="")
            self.Basic_info = open(self.Data_name02, "a", newline="")
            self.raw_writer = csv.writer(self.BMS_raw_file)
            self.basic_writer = csv.writer(self.Basic_info)
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
                ]
            )

            self.basic_writer = csv.writer(self.Basic_info)
            self.basic_writer.writerow(["V", "SOC", "SOH"])

            self.data_count_id = 1
            print("get self data count", self.data_count_id)

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
        # self.cha_flag = 1
        # self.cha_count = 1
        self.SOH = 100
        V_avg_list = [0 for i in range(10)]
        V_avg_flag = 0
        for i in range(10):
            # read BMS raw data for 1 second
            response_json = self.stmserial.readline()
            decode = json.loads(response_json)
            # print("get json data :\r\n",response_json)
            # print("decode json = ",decode)

            # decode V I data and set init data as input
            # self.V = decode["battery_voltage"] / (20 * 1000)
            # self.Q = decode["BMS1_AccumulatedCharge"] / (20 * 1000)  # wait point set
            # self.I = decode["BMS1_pack_current"] / 1000

            self.V = 2.5413
            self.Q = 70
            self.I = -50

            if V_avg_flag == 0:
                V_avg = self.V
                self.SOC = Data.cal_SOC(self, self.I, self.V, V_avg, self.SOH)
            # t = temperature,sort from high to low,use the higher as temp
            t = [decode["BMS1_TS1Temp"], decode["BMS1_TS3Temp"]]
            t = sorted(t, reverse=True)
            self.Temp = t[0] / 100

            # decode battery state
            if decode["battery_state"] == "OK":
                self.state = 0
                error_state = " "
                error_code = None
            else:
                self.state = 1
                error_state = Data.decode_err_code(decode["BMS_error_code"])
                error_code = decode["BMS_error_code"]

            self.reg = [
                self.state,
                error_code,
                decode["battery_voltage"],
                decode["BMS1_pack_current"],
                self.Temp,
                self.SOC,
                self.SOH,
                decode["cell_voltage0"],
                decode["cell_voltage1"],
                decode["cell_voltage2"],
                decode["cell_voltage3"],
                decode["cell_voltage4"],
                decode["cell_voltage5"],
                decode["cell_voltage6"],
                decode["cell_voltage7"],
                decode["cell_voltage8"],
                decode["cell_voltage9"],
                decode["cell_voltage10"],
                decode["cell_voltage11"],
                decode["cell_voltage12"],
                decode["cell_voltage13"],
                decode["cell_voltage14"],
                decode["cell_voltage15"],
                decode["cell_voltage16"],
                decode["cell_voltage17"],
                decode["cell_voltage18"],
                decode["cell_voltage19"],
            ]

            if i == 9:
                # cal SOC and record SOH data every 10 seconds
                V_avg_list[V_avg_flag] = self.V
                V_avg = Data.sliding_avg(V_avg_list, V_avg_flag)
                V_avg_flag += 1
                self.SOC = Data.cal_SOC(self, self.I, self.V, V_avg, self.SOH)
                print("id = ", self.data_count_id)
                now_time = datetime.now()

                SOH_state = Data.record_SOH_data(self, self.V, self.I, self.Q)
                if SOH_state == "cha_OK":
                    self.SOH = Data.cal_SOH(self)
                    pass
                else:
                    pass

                print("V=", self.V)
                print("I=", self.I)
                print("Temp=", self.Temp)
                print("SOC=", self.SOC)
                print("SOH=", self.SOH)
                print("\n")

                # record basic battery information every 10 seconds
                self.raw_writer.writerow(
                    [
                        self.data_count_id,
                        "BMS_1",
                        self.state,
                        error_state,
                        decode["battery_voltage"],
                        decode["BMS1_pack_current"],
                        self.Temp,
                        self.SOC,
                        self.SOH,
                        now_time,
                        decode["cell_voltage0"],
                        decode["cell_voltage1"],
                        decode["cell_voltage2"],
                        decode["cell_voltage3"],
                        decode["cell_voltage4"],
                        decode["cell_voltage5"],
                        decode["cell_voltage6"],
                        decode["cell_voltage7"],
                        decode["cell_voltage8"],
                        decode["cell_voltage9"],
                        decode["cell_voltage10"],
                        decode["cell_voltage11"],
                        decode["cell_voltage12"],
                        decode["cell_voltage13"],
                        decode["cell_voltage14"],
                        decode["cell_voltage15"],
                        decode["cell_voltage16"],
                        decode["cell_voltage17"],
                        decode["cell_voltage18"],
                        decode["cell_voltage19"],
                    ]
                )
            self.basic_writer.writerow([self.V, self.SOC, self.SOH])
            self.data_count_id += 1

            self.reg = [
                self.state,
                error_code,
                decode["battery_voltage"],
                decode["BMS1_pack_current"],
                self.Temp,
                self.SOC,
                self.SOH,
                decode["cell_voltage0"],
                decode["cell_voltage1"],
                decode["cell_voltage2"],
                decode["cell_voltage3"],
                decode["cell_voltage4"],
                decode["cell_voltage5"],
                decode["cell_voltage6"],
                decode["cell_voltage7"],
                decode["cell_voltage8"],
                decode["cell_voltage9"],
                decode["cell_voltage10"],
                decode["cell_voltage11"],
                decode["cell_voltage12"],
                decode["cell_voltage13"],
                decode["cell_voltage14"],
                decode["cell_voltage15"],
                decode["cell_voltage16"],
                decode["cell_voltage17"],
                decode["cell_voltage18"],
                decode["cell_voltage19"],
            ]

        time.sleep(0.1)


def Get_new_data(port, data_name1, data_name2):
    NEW_DATA = Data()
    NEW_DATA.serial_setup(port)
    NEW_DATA.open_file(data_name1, data_name2)
    NEW_DATA.read_record_raw_data()


if __name__ == "__main__":
    Get_new_data(
        "/dev/ttyUSB0",
        "./output_data/BMS1_Output_raw_data.csv",
        "./output_data/BMS1_Basic_data.csv",
    )
