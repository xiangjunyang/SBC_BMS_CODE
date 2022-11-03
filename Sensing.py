import csv
import json
import serial
import numpy as np
from datetime import datetime
from keras.models import load_model

class Data():
	def __init__(self):
		self.SOC_Cha_model_files = './models/Cha_model.h5'
		self.SOC_Discha_model_files = './models/Discha_model.h5'
		self.SOH_Cha_model_files = './models/Cha_model.h5'
		self.SOH_Discha_model_files = './models/Discha_model.h5'
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
		V = Data.Inverse_Single_input(V, 2.3169, 2.7019)
		T = Data.Inverse_Single_input(T, 25.7, 40.3)
		SOH = Data.Inverse_Single_input(SOH, 97.0, 102.4)
		
		V_in = np.full((1,1),V)
		T_in = np.full((1,1),T)
		SOH_in = np.full((1,1),SOH)
		if I > 0:
			SOC_pred = self.SOC_Discha_model.predict((V_in, T_in, SOH_in))
		else:
			SOC_pred = self.SOC_Cha_model.predict((V_in, T_in, SOH_in))
		SOC = Data.Reverse_Single_input(SOC_pred[0][0], 0, 100)
		return SOC

	def read_record_raw_data(self,Data_name):
		#open csv file
		BMS_file = open(Data_name, 'a+', newline='')
		
		#write init data
		reader = csv.reader(BMS_file)
		writer = csv.writer(BMS_file)
		col=reader.line_num
		print("get col =", col)
		
		data = []
		for row in reader:
			data.append(row)
		if col == 0:
			writer.writerow(['Data_id', 'BMS_Num', 'internal tempture(*C)', 'TS1_tempture(*C)', 'ts3 tempture(*C)', 'cell current(mA)', 'Stack Voltage(mV)', 'cell_voltage1', 'cell_voltage2', 'cell_voltage3', 'cell_voltage4', 'cell_voltage5', 'cell_voltage6', 'cell_voltage7', 'cell_voltage8', 'cell_voltage9', 'cell_voltage10', 'cell_voltage11', 'cell_voltage12', 'cell_voltage13', 'cell_voltage14', 'cell_voltage15', 'cell_voltage16', 'cell_voltage17', 'cell_voltage18', 'cell_voltage19', 'cell_voltage20', 'SOC', 'SOH', 'Current_Time'])
			data_count_id = 1
		else:
			print("get col =", col)
			data_count_id = reader[col][0]
			SOH = reader[col][28]	
		#pyserial init
		stmserial = serial.Serial()
		stmserial.port = "/dev/ttyUSB0"
		# 115200,N,8,1
		stmserial.baudrate = 115200
		stmserial.bytesize = serial.EIGHTBITS  # number of bits per bytes
		stmserial.parity = serial.PARITY_NONE  # set parity check
		stmserial.stopbits = serial.STOPBITS_ONE  # number of stop bits

		stmserial.timeout = 1.0  # non-block read 0.5s
		stmserial.writeTimeout = 0.5  # timeout for write 0.5s
		stmserial.xonxoff = False  # disable software flow control
		stmserial.rtscts = False  # disable hardware (RTS/CTS) flow control
		stmserial.dsrdtr = False  # disable hardware (DSR/DTR) flow control
		stmserial.open()

		for i in range(10):
			response_json = stmserial.readline()
			#print("get json data :\r\n",response_json)
			decode = json.loads(response_json)
			print("decode json = ",decode)
			V=decode["Stack_Voltage"]
			I=decode["pack_current"]
			Temp=decode["internalTemp"]
			Pre_SOC = Data.cal_SOC(self,I, V, Temp, 100)
			print("pre soc = ",Pre_SOC)

			Current_Time = datetime.now()
			writer.writerow([data_count_id, 'bms 1', decode["internalTemp"], decode["TS1Temp"], decode["TS3Temp"], decode["pack_current"], decode["Stack_Voltage"], decode["cell_voltage1"], decode["cell_voltage2"], decode["cell_voltage3"], decode["cell_voltage4"], decode["cell_voltage5"], decode["cell_voltage6"], decode["cell_voltage7"], decode["cell_voltage8"], decode["cell_voltage9"], decode["cell_voltage10"], decode["cell_voltage11"], decode["cell_voltage12"], decode["cell_voltage13"], decode["cell_voltage14"], decode["cell_voltage15"], decode["cell_voltage16"], decode["cell_voltage17"], decode["cell_voltage18"], decode["cell_voltage19"], decode["cell_voltage20"], 100, 100, Current_Time])
			data_count_id += 1
		BMS_file.close()
def main():
	NEW_DATA=Data()
	NEW_DATA.read_record_raw_data("./output_data/BMS1_output_data") 

if __name__ == '__main__':
	main()
