import threading
import time

from Get_Data import *
from Modbus_output  import *

if __name__ == '__main__':
	t = threading.Thread(target = Get_new_data('/dev/ttyUSB1',"./output_data/BMS1_output_data"))
	#t = threading.Thread(target = Start_Modbus('/dev/ttyUSB0'))
	t.start()

	Start_Modbus('/dev/ttyUSB0')
	#Get_new_data('/dev/ttyUSB1',"./output_data/BMS1_output_data")
	t.join()

	print("finish thread")

