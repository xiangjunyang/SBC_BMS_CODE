
'''
python version :3.9.6 64bit
library version:pyserial 3.5
check port list command:python -m serial.tools.list_ports

ref link:
pyserial:https://pyserial.readthedocs.io/en/latest/tools.html
decode json:https://officeguide.cc/python-read-write-json-encode-decode-tutorial-examples/
Multithreading:https://blog.gtwang.org/programming/python-threading-multithreaded-programming-tutorial/
pylibmodbus:https://blog.csdn.net/ethercat_i7/article/details/104543724

'''

import csv
import datetime
import json
# internal module include
import time
from xml.dom import ValidationErr

import numpy as np
# external module include
import serial
from keras.models import Model, Sequential, load_model

SOC_Cha_model_files = './model_output/SOC_Cha_model.h5'
SOC_Discha_model_files = './model_output/SOC_Discha_model.h5'
SOH_Cha_model_files = './model_output/Cha_model.h5'
SOH_Discha_model_files = './model_output/Discha_model.h5'
SOc_Cha_model = load_model(SOC_Cha_model_files)
SOC_Discha_model = load_model(SOC_Discha_model_files)
SOH_Cha_model = load_model(SOH_Cha_model_files)
SOH_Discha_model = load_model(SOH_Discha_model_files)


def check_emergency(eme_state):
    if eme_state == 1:
        print("eme on")
    else:
        print("eme off")


def Inverse_1D(inputs, lower_limit, upper_limit):
    output = np.zeros(inputs.shape[0])
    for i in range(inputs.shape[0]):
        output[i] = (abs(inputs[i]-lower_limit)/abs(upper_limit-lower_limit))
    return output


def Reverse_1D(inputs, lower_limit, upper_limit):
    output = np.zeros(inputs.shape[0])
    for i in range(inputs.shape[0]):
        output[i] = ((inputs[i])*abs(upper_limit-lower_limit))+lower_limit
    return output


def Inverse_Single_input(inputs, lower_limit, upper_limit):
    output = (abs(inputs-lower_limit)/abs(upper_limit-lower_limit))
    return output


def Reverse_Single_input(inputs, lower_limit, upper_limit):
    output = (abs(upper_limit-lower_limit)*inputs)+lower_limit
    return output


def cal_SOC(I, V, T, SOH):
    V = Inverse_Single_input(V, 2.3169, 2.7019)
    T = Inverse_Single_input(T, 25.7, 40.3)
    SOH = Inverse_Single_input(SOH, 97.0, 102.4)

    if I > 0:
        SOC_pred = SOC_Discha_model.predict((V, T, SOH))
    else:
        SOC_pred = SOc_Cha_model.predict((V, T, SOH))
    SOC = Reverse_Single_input(SOC_pred, 0, 100)

    return SOC


def cal_SOH(I, dVdT, dSocdV):
    # dVdT = Inverse_1D(dVdT, 2.3169, 2.7019)
    # dSocdV = Inverse_1D(dSocdV, 25.7, 40.3)

    if I > 0:
        SOH_pred = SOH_Discha_model.predict((dVdT, dSocdV))
    else:
        SOH_pred = SOH_Cha_model.predict((dVdT, dSocdV))
    SOH = Reverse_Single_input(SOH_pred, 97.0, 102.4)
    return SOH


def read_record_raw_data(Data_name, SOH):
    # # 開啟輸出的 CSV 檔案
    BMS_Data = open(Data_name, 'w', newline='')
    # 寫入初始資料
    writer = csv.writer(BMS_Data)
    writer.writerow(['BMS_Num', 'internal tempture(*C)', 'TS1_tempture(*C)', 'ts3 tempture(*C)', 'cell current(mA)', 'Stack Voltage(mV)', 'cell_voltage1', 'cell_voltage2', 'cell_voltage3', 'cell_voltage4', 'cell_voltage5', 'cell_voltage6', 'cell_voltage7',
                    'cell_voltage8', 'cell_voltage9', 'cell_voltage10', 'cell_voltage11', 'cell_voltage12', 'cell_voltage13', 'cell_voltage14', 'cell_voltage15', 'cell_voltage16', 'cell_voltage17', 'cell_voltage18', 'cell_voltage19', 'cell_voltage20', 'SOC', 'SOH', 'Current_Time'])
    # writer.writerow(['data example', 26.5, 28.5, 28.9, 500, 24050])

    # pyserial init
    stmserial = serial.Serial()
    stmserial.port = "COM7"
    # 115200,N,8,1
    stmserial.baudrate = 115200
    stmserial.bytesize = serial.EIGHTBITS  # number of bits per bytesj
    stmserial.parity = serial.PARITY_NONE  # set parity check
    stmserial.stopbits = serial.STOPBITS_ONE  # number of stop bits

    stmserial.timeout = 0.5  # non-block read 0.5s
    stmserial.writeTimeout = 0.5  # timeout for write 0.5s
    stmserial.xonxoff = False  # disable software flow control
    stmserial.rtscts = False  # disable hardware (RTS/CTS) flow control
    stmserial.dsrdtr = False  # disable hardware (DSR/DTR) flow control
    stmserial.open()

    while(1):
        response_json = stmserial.readline()
        decode = json.loads(response_json)
        print("decode json =", decode)
        print("internal temp =", decode["internalTemp"])

        V = decode["Stack_Voltage"]
        Temp = decode["internalTemp"]
        I = decode["pack_current"]

        dVdT, dSocdV = 0
        Pre_SOC = cal_SOC(I, V, Temp, SOH)
        Pre_SOH = cal_SOH(I, dVdT, dSocdV)

        Current_Time = datetime.datetime.now()

        # write 10 cell data
        # writer.writerow(['bms 1', decode["internalTemp"], decode["TS1Temp"], decode["TS3Temp"], decode["pack_current"], decode["Stack_Voltage"], decode["cell_voltage1"], decode["cell_voltage2"],decode["cell_voltage3"], decode["cell_voltage4"], decode["cell_voltage5"], decode["cell_voltage6"], decode["cell_voltage7"], decode["cell_voltage8"], decode["cell_voltage9"], decode["cell_voltage10"]])

        # write 20 cell data
        writer.writerow(['bms 1', decode["internalTemp"], decode["TS1Temp"], decode["TS3Temp"], decode["pack_current"], decode["Stack_Voltage"], decode["cell_voltage1"], decode["cell_voltage2"], decode["cell_voltage3"], decode["cell_voltage4"], decode["cell_voltage5"], decode["cell_voltage6"],
                        decode["cell_voltage7"], decode["cell_voltage8"], decode["cell_voltage9"], decode["cell_voltage10"], decode[""], decode[""], decode[""], decode[""], decode[""], decode[""], decode[""], decode[""], decode[""], decode[""], decode[""], Pre_SOC, Pre_SOH, Current_Time])
        print('get json data:\r\n', response_json)
        time.sleep(0.3)


def Modbus_return_by_SunSpen(I, V, T, SOC, SOH):
    reg = [V, I, T, SOC, SOH]

    return SOH
