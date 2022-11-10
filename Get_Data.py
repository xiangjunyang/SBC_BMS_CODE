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

	def sliding_avg(inputs, window_size,n_th):
		output=[]
		for i in range(len(inputs)-window_size):
			avg=0
			for j in range(i,i+window_size,+1):
				avg+=inputs[j]
				avg=avg/window_size
				output.append(avg**n_th)    
		return output
	
	def cal_SOH(self,file,mode):
		file = open(file)
		reader = csv.DictReader(file)
		dVdT=[]
		dSocdV=[]
		for row in reader:
			dVdT.append(row['dV_dt'])
			dSocdV.append(row['dSOC_dv'])
		
		dVdT_arr   = np.array(dVdT)
		dSocdV_arr = np.array(dSocdV)
		dVdT_arr = dVdT_arr.astype(np.float)
		dSocdV_arr = dSocdV_arr.astype(np.float)	
	
		dVdT_arr   = Data.sliding_avg(dVdT_arr,20,1)
		dSocdV_arr = Data.sliding_avg(dSocdV_arr, 20,1)
		
		dVdT_arr = Data.Inverse_1D(dVdT_arr, 2.3169, 2.7019)
		dSocdV_arr = Data.Inverse_1D(dSocdV_arr, 25.7, 40.3)
		
		dVdT_arr=np.full((1,dVdT_arr.shape[0]), dVdT_arr) 
		dSocdV_arr=np.full((1,dSocdV_arr.shape[0]), dSocdV_arr)			

		if mode == 'dis_OK':
			SOH_pred = self.SOH_Discha_model.predict((dVdT_arr, dSocdV_arr))
		elif mode == 'cha_OK':	
			SOH_pred = self.SOH_Cha_model.predict((dVdT_arr, dSocdV_arr))

		SOH = Data.Reverse_Single_input(SOH_pred[0][0], 97.0, 102.4)
		return SOH

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

	def record_SOH_data(self,V,I,now_time,Pre_SOC):	
		if I>0 and 2.65>=V and V>=2.3:
			if self.dis_flag==1:
				self.old_time=now_time
				self.old_v=V
				self.old_soc=Pre_SOC
			elif self.dis_flag%10==0:
				dt=(now_time-self.old_time).total_seconds()
				dv=(V-self.old_v)
				dsoc=Pre_SOC-self.old_soc
				dvdt=dv/dt
				dsocdv=dsoc/dv
				self.SOH_writer.writerow(['discha',dvdt,dsocdv])
				self.old_time=now_time
				self.old_v=V
				self.old_soc=Pre_SOC
				self.dis_count+=1
			self.dis_flag+=1
			if self.dis_count==254:
				#deal_SOH_data('discha')
				self.dis_count=1
				return 'dis_OK'
			else:
				return 'dis_unready'
		
		elif I<0 and 2.65>=V and V>=2.45:
			if self.cha_flag==1:
				self.old_time=now_time
				self.old_v=V
				self.old_soc=Pre_SOC
			elif self.cha_flag%10==0:
				dt=(now_time-self.old_time).total_seconds()
				dv=(V-self.old_v)
				dsoc=Pre_SOC-self.old_soc
				dvdt=dv/dt
				dsocdv=dsoc/dv
				self.SOH_writer.writerow(['cha',dvdt,dsocdv])
				self.old_time=now_time
				self.old_v=V
				self.old_soc=Pre_SOC
				self.cha_count+=1
			self.cha_flag+=1
			if self.cha_count==106:
				#deal_SOH_data('cha')
				self.cha_count=1
				return 'cha_OK'
			else:
				return 'cha_unready'
	
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
	
	def open_file(self,Data_name01,Data_name02,Data_name03):
		self.Data_name01=Data_name01
		self.Data_name02=Data_name02
		self.Data_name03=Data_name03
		#open csv file
		self.BMS_raw_file = open(self.Data_name01, 'w+', newline='')
		self.Basic_info = open(self.Data_name02,'w+', newline='')
		self.SOH_raw_file = open(self.Data_name03,'w+', newline='')
		#write init data
		self.raw_writer = csv.writer(self.BMS_raw_file)		
		self.raw_writer.writerow(['Data_id', 'BMS_Num', 'state', 'error code', 'Stack Voltage(mV)', 'cell current(mA)', ' tempture(*C)', 'SOC', 'SOH', 'Current_Time',  'cell_voltage1', 'cell_voltage2', 'cell_voltage3', 'cell_voltage4', 'cell_voltage5', 'cell_voltage6', 'cell_voltage7', 'cell_voltage8', 'cell_voltage9', 'cell_voltage10', 'cell_voltage11', 'cell_voltage12', 'cell_voltage13', 'cell_voltage14', 'cell_voltage15', 'cell_voltage16', 'cell_voltage17', 'cell_voltage18', 'cell_voltage19', 'cell_voltage20'])
		
		self.basic_writer = csv.writer(self.Basic_info)	
		self.basic_writer.writerow(['V','SOC','SOH'])
		
		self.SOH_writer = csv.writer(self.SOH_raw_file)
		self.SOH_writer.writerow(['mode','dV_dt','dSOC_dv'])
		
		self.data_count_id = 1
		print("get self data count",self.data_count_id)

	def read_record_raw_data(self):
		self.dis_flag=1
		self.dis_count=1
		self.cha_flag=1
		self.cha_count=1
		self.SOH=100
		while(1):
			response_json = self.stmserial.readline()
			#print("get json data :\r\n",response_json)
			decode = json.loads(response_json)
			#print("decode json = ",decode)
			self.V=decode["Stack_Voltage"]
			self.I=decode["pack_current"]
			self.Temp=decode["internalTemp"]
			self.SOC = Data.cal_SOC(self,I, V, Temp, SOH)
			#print("pre soc = ",Pre_SOC)
			print("id = ",self.data_count_id)
			now_time = datetime.now()
			
			SOH_state=Data.record_SOH_data(self,V,I,now_time,self.SOC) 
			if SOH_state == 'cha_OK' or SOH_state == 'dis_OK':
				self.BMS_raw_file.close()
				self.Basic_info.close()  
				self.SOH_raw_file.close()
				self.SOH = Data.cal_SOH(self,self.Data_name03,SOH_state)
				self.open_file(self.Data_name01,self.Data_name02,self.Data_name03)
				pass
			else :	
				pass
			
			print("SOH=",self.SOH)
			self.raw_writer.writerow([self.data_count_id, 'bms_1', 'state', 'error_code', decode["Stack_Voltage"],  decode["pack_current"], decode["internalTemp"],self.SOC, self.SOH, now_time, decode["cell_voltage1"], decode["cell_voltage2"], decode["cell_voltage3"], decode["cell_voltage4"], decode["cell_voltage5"], decode["cell_voltage6"], decode["cell_voltage7"], decode["cell_voltage8"], decode["cell_voltage9"], decode["cell_voltage10"], decode["cell_voltage11"], decode["cell_voltage12"], decode["cell_voltage13"], decode["cell_voltage14"], decode["cell_voltage15"], decode["cell_voltage16"], decode["cell_voltage17"], decode["cell_voltage18"], decode["cell_voltage19"], decode["cell_voltage20"] ])
			self.data_count_id += 1
			
			self.reg=['state', 'error_code', decode["Stack_Voltage"],  decode["pack_current"], decode["internalTemp"],self.SOC, self.SOH, decode["cell_voltage1"], decode["cell_voltage2"], decode["cell_voltage3"], decode["cell_voltage4"], decode["cell_voltage5"], decode["cell_voltage6"], decode["cell_voltage7"], decode["cell_voltage8"], decode["cell_voltage9"], decode["cell_voltage10"], decode["cell_voltage11"], decode["cell_voltage12"], decode["cell_voltage13"], decode["cell_voltage14"], decode["cell_voltage15"], decode["cell_voltage16"], decode["cell_voltage17"], decode["cell_voltage18"], decode["cell_voltage19"], decode["cell_voltage20"] ]
			time.sleep(0.5)

def Get_new_data(port,data_name1,data_name2,data_name3):
	NEW_DATA=Data()
	NEW_DATA.serial_setup(port)
	NEW_DATA.open_file(data_name1,data_name2,data_name3)
	NEW_DATA.read_record_raw_data() 

if __name__ == '__main__':
	Get_new_data('/dev/ttyUSB1',"./output_data/BMS1_Output_raw_data","./output_data/BMS1_Basic_data","./output_data/BMS1_SOH_raw_data")
