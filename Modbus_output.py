import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import serial
import time  # add this

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
	def Start_service(self):
		print("Modbus service start")
		try:
			self.modbusServ.start()
			print("receiver module started")

			slave = self.modbusServ.add_slave(1)
			slave.add_block ("b",cst.HOLDING_REGISTERS,0,4)
			slave.set_values("b",0,range(4))
			while True:
				slave.set_values ("b", 0, 11)      #0x0b
				slave.set_values ("b", 1, 36)	     #0x24
				slave.set_values ("b", 2, 0)
				slave.set_values ("b", 3, 0)
	
				time.sleep(0.5)   # small delay to let the communication thread doing his job

		finally:
			self.modbusServ.stop()
			print("Modbus service stop")

def Start_Modbus(port):
	mbs=Modbus_output(port)
	mbs.Start_service()

if __name__ == '__main__':
	Start_Modbus('/dev/ttyUSB0')

