import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import serial
import time  # add this
import numpy as np
from SunSpec_mapping import *

slave_address = 17
Module_Index = 1

class Modbus_output():
	def __init__(self,PORT):
	#PORT = '/dev/ttyUSB0'
		self.modbusServ = modbus_rtu.RtuServer(
                                    serial.Serial(PORT),
                                    baudrate=9600, 
                                    bytesize=8, 
                                    parity='N', 
                                    stopbits=1, 
                                    xonxoff=0
                                    )
	
	def write_reg_data(self,reg):
		self.state = reg[0]
		self.err_code = reg[1]
		self.V = reg[2]
		self.I = reg[3]
		self.Temp = reg[4]
		self.SOC = reg[5]
		self.SOH = reg[6]
		self.Voltage=[]
		for i in range(20):
			self.Voltage.append(reg[i+6])
		self.V_arr=np.array(self.Voltage)
		self.Vmax=np.amax(self.V_arr)
		self.Vmaxindex=np.argmax(self.V_arr)
		self.Vmin=np.amin(self.V_arr)
		self.Vminindex=np.argmin(self.V_arr)
		#self.V1 = reg[7]
	
	def Start_service(self,reg):
		#print("Modbus service start")
		try:
			Suns=Suns_map()
			self.modbusServ.start()
			#print("receiver module started")
			Modbus_output.write_reg_data(self,reg)	
			
			Suns.SunS_mapping(slave_address,Module_Index,self.I,self.V_arr,self.Temp,self.state,self.err_code,self.SOC,self.SOH,self.V,self.Vmax,self.Vmin,self.Vmaxindex,self.Vminindex)
			
			slave = self.modbusServ.add_slave(slave_address)
			slave.add_block ("mbs",cst.HOLDING_REGISTERS,40000,300)
			
            reg=list(Suns.SunSp)
            slave.set_values ("mbs", 40000,reg)      #0x0b
	
            time.sleep(0.5)   # small delay to let the communication thread doing his job
			while True:
                pass
                #reg=list(Suns.SunSp)
                #slave.set_values ("mbs", 40000,reg)      #0x0b
	
				#time.sleep(0.5)   # small delay to let the communication thread doing his job

		finally:
			self.modbusServ.stop()
			print("Modbus service stop")

def Start_Modbus(port):
	mbs=Modbus_output(port)
	mbs.Start_service()

if __name__ == '__main__':
	Start_Modbus('/dev/ttyUSB0')

