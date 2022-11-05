import threading
import time

from Get_Data import *
from Modbus_output  import *

def  main():
	
	serial_port='/dev/ttyUSB1'
	modbus_port='/dev/ttyUSB0'
	Output_data="./output_data/BMS1_output_data"

	Get_data=Data()
	Get_data.serial_setup(serial_port)
	Get_data.open_file(Output_data)

	mbs_service=Modbus_output(modbus_port)
	
	#thread task start	
	t = threading.Thread(target = Get_data.read_record_raw_data)
		
	t.start()
	
	mbs_service.Start_service()
	
	#wait the thread finish
	t.join()

	print("finish thread")


if __name__ == '__main__':	
	main()
