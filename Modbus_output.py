import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import serial
import time  # add this
import numpy as np
from SunSpec_mapping import *

slave_address = 17
Module_Index = 1


class Modbus_output:
    def __init__(self, PORT):
        # PORT = '/dev/ttyUSB0'
        self.modbusServ = modbus_rtu.RtuServer(
            serial.Serial(PORT),
            baudrate=9600,
            bytesize=8,
            parity="N",
            stopbits=1,
            xonxoff=0,
        )

    def write_reg_data(self, reg):
        self.state = reg[0]
        self.err_code = reg[1]
        self.V = reg[2]
        self.I = reg[3]
        self.Temp = reg[4]
        self.SOC = reg[5]
        self.SOH = reg[6]
        self.Voltage = []
        for i in range(20):
            self.Voltage.append(reg[i + 7])
        self.V_arr = np.array(self.Voltage)
        # print(self.V_arr)
        self.Vmax = np.amax(self.V_arr)
        self.Vmaxindex = np.argmax(self.V_arr)
        self.Vmin = np.amin(self.V_arr)
        self.Vminindex = np.argmin(self.V_arr)
        # self.V1 = reg[7]

    def Start_service(self):
        print("Modbus service start")
        self.modbusServ.start()

        self.slave = self.modbusServ.add_slave(slave_address)
        self.slave.add_block("mbs", cst.HOLDING_REGISTERS, 40000, 300)

    def Stop_service(self):
        self.modbusServ.stop()
        print("Modbus service stop")

    def Write_holding_register(self, reg):
        Suns = Suns_map()
        print("receiver module started")
        Modbus_output.write_reg_data(self, reg)
        Suns.SunS_mapping(
            slave_address,
            Module_Index,
            self.I,
            self.V_arr,
            self.Temp,
            self.state,
            self.err_code,
            self.SOC,
            self.SOH,
            self.V,
            self.Vmax,
            self.Vmin,
            self.Vmaxindex,
            self.Vminindex,
        )
        reg = list(Suns.SunSp)
        self.slave.set_values("mbs", 40000, reg)
        time.sleep(0.1)  # small delay to let the communication thread doing his job
        while True:
            time.sleep(0.1)
            break
            # pass


def Start_Modbus(port, reg):
    mbs = Modbus_output(port)
    mbs.Start_service()
    # mbs.Write_holding_register(reg)
    while 1:
        mbs.Write_holding_register(reg)
    mbs.Stop_service()


if __name__ == "__main__":
    reg = [-50 for i in range(27)]
    Start_Modbus("/dev/ttyUSB1", reg)
