import csv
import json
import serial
from datetime import datetime

class Data():
	def read_record_raw_data(Data_name):
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
			#print("decode json = ",decode)
			Current_Time = datetime.now()
			writer.writerow([data_count_id, 'bms 1', decode["internalTemp"], decode["TS1Temp"], decode["TS3Temp"], decode["pack_current"], decode["Stack_Voltage"], decode["cell_voltage1"], decode["cell_voltage2"], decode["cell_voltage3"], decode["cell_voltage4"], decode["cell_voltage5"], decode["cell_voltage6"], decode["cell_voltage7"], decode["cell_voltage8"], decode["cell_voltage9"], decode["cell_voltage10"], decode["cell_voltage11"], decode["cell_voltage12"], decode["cell_voltage13"], decode["cell_voltage14"], decode["cell_voltage15"], decode["cell_voltage16"], decode["cell_voltage17"], decode["cell_voltage18"], decode["cell_voltage19"], decode["cell_voltage20"], 100, 100, Current_Time])
			data_count_id += 1
		BMS_file.close()
def main():
	Data.read_record_raw_data("./output_data/BMS1_output_data") 

if __name__ == '__main__':
	main()
