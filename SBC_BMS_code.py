import threading
import time

from Get_Data import *
from Modbus_output import *


def main():

    serial_port = "/dev/ttyUSB0"
    modbus_port = "/dev/ttyUSB1"
    Output_raw_data = "/home/orangepi/Desktop/project/SBC_BMS_CODE/output_data/BMS1_Output_raw_data.csv"
    BMS_Basic_data = "/home/orangepi/Desktop/project/SBC_BMS_CODE/output_data/BMS1_Basic_data.csv"
    # SOH_raw_data="./output_data/BMS1_SOH_raw_data.csv"

    global Get_data
    Get_data = Data()
    Get_data.serial_setup(serial_port)
    Get_data.open_file(Output_raw_data, BMS_Basic_data)

    mbs_service = Modbus_output(modbus_port)
    mbs_service.Start_service()

    while 1:
        # thread task start
        output_data = Get_data.reg
        t = threading.Thread(target=Get_data.read_record_raw_data)
        # print("finish get new data",output_data)
        t.start()

        mbs_service.Write_holding_register(output_data)
        # print("finish write reg data")
        t.join()

        # ##################
        # output_data = Get_data.reg
        # Get_data.read_record_raw_data()
        # mbs_service.Write_holding_register(output_data)

    mbs_service.Stop_service()


if __name__ == "__main__":
    main()
