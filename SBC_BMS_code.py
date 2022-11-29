import threading
import time

from Get_Data import *
from Modbus_output  import *

def  main():
	
    serial_port='/dev/ttyUSB1'
    modbus_port='/dev/ttyUSB0'
    Output_raw_data="./output_data/BMS1_output_raw_data.csv"
    BMS_Basic_data="./output_data/BMS1_Basic_data.csv"
    SOH_raw_data="./output_data/BMS1_SOH_raw_data.csv"
    
    Get_data=Data()
    Get_data.serial_setup(serial_port)
    Get_data.open_file(Output_raw_data,BMS_Basic_data,SOH_raw_data)
    
    mbs_service=Modbus_output(modbus_port)
    
    #thread task start	
    t = threading.Thread(target = Get_data.read_record_raw_data)
    	
    t.start()
    
    mbs_service.Start_service(Get_data.reg)
    
    #wait the thread finish
    t.join()
    
    print("finish thread")


if __name__ == '__main__':	
	main()
