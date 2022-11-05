import csv
import json
import serial
import time
import numpy as np
from datetime import datetime
from keras.models import load_model

class Data():
	def __init__(self):
		self.SOC_Cha_model_files = './models/SOC_Cha_model.h5'
		self.SOC_Discha_model_files = './models/SOC_Discha_model.h5'
		self.SOH_Cha_model_files = './models/SOH_Cha_model.h5'
		self.SOH_Discha_model_files = './models/SOH_Discha_model.h5'
		self.SOC_Cha_model = load_model(self.SOC_Cha_model_files)
		self.SOC_Discha_model = load_model(self.SOC_Discha_model_files)
		self.SOH_Cha_model = load_model(self.SOH_Cha_model_files)
		self.SOH_Discha_model = load_model(self.SOH_Discha_model_files)
		print("load model successfully")

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


	def cal_SOC(self,I, V, T, SOH):
		V = Data.Inverse_Single_input(V, 2.3169, 2.7019)#V
		T = Data.Inverse_Single_input(T, 25.7, 40.3)#T
		SOH = Data.Inverse_Single_input(SOH, 97.0, 102.4)#SOH
		
		V_in = np.full((1,1),0.78415105)#V
		T_in = np.full((1,1),0.34246575)#T
		SOH_in = np.full((1,1),0.98148148)#SOH
		if I > 0:
			SOC_pred = self.SOC_Discha_model.predict((V_in, T_in, SOH_in))
			print("discharge mode")
		else:
			SOC_pred = self.SOC_Cha_model.predict((V_in, T_in, SOH_in))
			print("charge mode")
		SOC = Data.Reverse_Single_input(SOC_pred[0][0], 0, 100)
		return SOC

	def serial_setup(self,PORT):
		# pyserial init
		self.stmserial = serial.Serial()
		self.stmserial.port = PORT
		# 115200,N,8,1
		self.stmserial.baudrate = 115200
		self.stmserial.bytesize = serial.EIGHTBITS  # number of bits per bytes
		self.stmserial.parity = serial.PARITY_NONE  # set parity check
		self.stmserial.stopbits = serial.STOPBITS_ONE  # number of stop bits

		self.stmserial.timeout = 1.0  # non-block read 0.5s
		self.stmserial.writeTimeout = 0.5  # timeout for write 0.5s
		self.stmserial.xonxoff = False  # disable software flow control
		self.stmserial.rtscts = False  # disable hardware (RTS/CTS) flow control
		self.stmserial.dsrdtr = False  # disable hardware (DSR/DTR) flow control
		self.stmserial.open()	
	
	def open_file(self,Data_name):
		#open csv file
		self.BMS_file = open(Data_name, 'a+', newline='')
		
		#write init data
		self.writer = csv.writer(self.BMS_file)		
		self.writer.writerow(['Data_id', 'BMS_Num', 'internal tempture(*C)', 'TS1_tempture(*C)', 'ts3 tempture(*C)', 'cell current(mA)', 'Stack Voltage(mV)', 'cell_voltage1', 'cell_voltage2', 'cell_voltage3', 'cell_voltage4', 'cell_voltage5', 'cell_voltage6', 'cell_voltage7', 'cell_voltage8', 'cell_voltage9', 'cell_voltage10', 'cell_voltage11', 'cell_voltage12', 'cell_voltage13', 'cell_voltage14', 'cell_voltage15', 'cell_voltage16', 'cell_voltage17', 'cell_voltage18', 'cell_voltage19', 'cell_voltage20', 'SOC', 'SOH', 'Current_Time'])
		self.data_count_id = 1
		print("get self data count",self.data_count_id)

	def read_record_raw_data(self):
		while(1):
			response_json = self.stmserial.readline()
			#print("get json data :\r\n",response_json)
			decode = json.loads(response_json)
			#print("decode json = ",decode)
			V=decode["Stack_Voltage"]
			I=decode["pack_current"]
			Temp=decode["internalTemp"]
			Pre_SOC = Data.cal_SOC(self,I, V, Temp, 100)
			print("pre soc = ",Pre_SOC)
			print(self.data_count_id)

			Current_Time = datetime.now()
			self.writer.writerow([self.data_count_id, 'bms 1', decode["internalTemp"], decode["TS1Temp"], decode["TS3Temp"], decode["pack_current"], decode["Stack_Voltage"], decode["cell_voltage1"], decode["cell_voltage2"], decode["cell_voltage3"], decode["cell_voltage4"], decode["cell_voltage5"], decode["cell_voltage6"], decode["cell_voltage7"], decode["cell_voltage8"], decode["cell_voltage9"], decode["cell_voltage10"], decode["cell_voltage11"], decode["cell_voltage12"], decode["cell_voltage13"], decode["cell_voltage14"], decode["cell_voltage15"], decode["cell_voltage16"], decode["cell_voltage17"], decode["cell_voltage18"], decode["cell_voltage19"], decode["cell_voltage20"], Pre_SOC, 100, Current_Time])
			self.data_count_id += 1
			time.sleep(0.3)

def Get_new_data(port,data_name):
	NEW_DATA=Data()
	NEW_DATA.serial_setup(port)
	NEW_DATA.open_file(data_name)
	NEW_DATA.read_record_raw_data() 

if __name__ == '__main__':
	Get_new_data('/dev/ttyUSB1',"./output_data/BMS1_output_data")
