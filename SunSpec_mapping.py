import numpy as np

class Suns_map():
	def SunS_mapping(self,Slave_Address ,Module_Index,current, voltage, temperature, state, error_code, SOC, SOH, total_voltage, max_vol, min_vol, max_vol_num, min_vol_num):

		self.SunSp = np.zeros(300, dtype='uint16')
		# ************************************#
		# SunSpec Identifier
		self.SunSp[0] = 'S' << 8 | 'u'
	    	self.SunSp[1] = 'n' << 8 | 'S'
	
		#************************************#
		#SunSpec Model Common #1 #
		self.SunSp[2] = 1
		self.SunSp[3] = 66
		
		# Manufacturer #16
		self.SunSp[4] = 'N' << 8 | 'T'
		self.SunSp[5] = 'U' << 8 | 'T'
		self.SunSp[6] = 'L' << 8 | 'A'
		self.SunSp[7] = 'B' << 8 | '3'
		self.SunSp[8] = '1' << 8 | '4'
		
		# Model #16
		self.SunSp[20] = 'L' << 8 | 'i'
		self.SunSp[21] = 't' << 8 | 'h'
		self.SunSp[22] = 'i' << 8 | 'u'
		self.SunSp[23] = 'm' << 8 | '-'
		self.SunSp[24] = 'I' << 8 | 'o'
		self.SunSp[25] = 'n' << 8 | ' '
		self.SunSp[26] = 'B' << 8 | 'a'
		self.SunSp[27] = 't' << 8 | 't'
		self.SunSp[28] = 'e' << 8 | 'r'
		self.SunSp[29] = 'y'
		
		# Options #8
		self.SunSp[36] = 'X' << 8 | 'i'
		self.SunSp[37] = 'a' << 8 | 'n'
		self.SunSp[38] = 'g' << 8 | '-'
		self.SunSp[39] = 'Y' << 8 | 'a'
		self.SunSp[40] = 'n' << 8 | 'g'
		
		# Version #8
		self.SunSp[44] = 'V' << 8 | 'e'
		self.SunSp[45] = 'r' << 8 | ' '
		self.SunSp[46] = '1' << 8 | '.'
		self.SunSp[47] = '0'
		
		# Serial Number #16
		self.SunSp[52] = 'N' << 8 | 'T'
		self.SunSp[53] = 'U' << 8 | 'T'
		self.SunSp[54] = '0' << 8 | '0'
		self.SunSp[55] = '0' << 8 | '1'
		
		# Device Address #1
		self.SunSp[68] = Slave_Address
		
		# Pad #1
		self.SunSp[69] = 0xffff
		
		#************************************#
		#SunSpec Model Battery Base Model #802 #
		self.SunSp[70] = 802
		self.SunSp[71] = 62
		
		# Nameplate Charge Capacity #1
		self.SunSp[72] = 25
		
		# Nameplate Energy Capacity #1
		self.SunSp[73] = 1365
		
		# Nameplate Max Charge Rate #1
		self.SunSp[74] = 90
		
		# Namplate Max Discharge Rate #1
		self.SunSp[75] = 90
		
		# Self Discharge Rate #1
		self.SunSp[76] = 10
		
		# Nameplate Max SoC #1
		self.SunSp[77] = 80
		
		# Nameplate Min SoC #1
		self.SunSp[78] = 20
		
		# Max Reserve Percent #1
		self.SunSp[79] = 80
		
		# Min Reserve Percent #1
		self.SunSp[80] = 20
		
		# State of Charge #1
		self.SunSp[81] = SOC
		
		# Depth of Discharge #1
		self.SunSp[82] = (100-SOC)
		
		# State of Health #1
		self.SunSp[83] = SOH
		
		# Cycle Count #2
		self.SunSp[84] = 0xffff
		self.SunSp[85] = 1
		
		# Charge Status #1
		self.SunSp[86] = 6
		
		# Control Mode #1
		self.SunSp[87] = 1
		
		# Battery Heartbeat #1
		self.SunSp[88] = 1
		
		# Controller Heartbeat #1
		self.SunSp[89] = 1
		
		# Alarm Reset #1
		self.SunSp[90] = 0
		
		# Battery Type #1
		self.SunSp[91] = 4
		
		# State of the Battery Bank #1
		self.SunSp[92] = 4
		
		# Vendor Battery Bank State #1
		self.SunSp[93] = 4
		
		# Warranty Date #2
		self.SunSp[94] = 2022
		self.SunSp[95] = 216
		
		# Battery Event 1 Bitfield #2
		self.SunSp[96] = 0xffff
		self.SunSp[97] = 0xffff
		# Battery Event 2 Bitfield #2
		self.SunSp[98] = 0xffff
		self.SunSp[99] = 0xffff
		# Vendor Event Bitfield 1  #2
		self.SunSp[100] = 0xffff
		self.SunSp[101] = 0xffff
		# Vendor Event Bitfield 2  #2
		self.SunSp[102] = 0xffff
		self.SunSp[103] = 0xffff
		
		# External Battery Voltage #1
		self.SunSp[104] = (total_voltage)
		
		# Max Battery Voltage #1
		self.SunSp[105] = (total_voltage)
		
		# Min Battery Voltage #1
		self.SunSp[106] = (total_voltage)
		
		# Max Cell Voltage #1
		self.SunSp[107] = max_vol
		
		# Max Cell Voltage String #1
		self.SunSp[108] = 0
		
		# Max Cell Voltage Module #1
		self.SunSp[109] = Module_Index
		
		# Min Cell Voltage #1
		self.SunSp[110] = min_vol
		
		# Min Cell Voltage String #1
		self.SunSp[111] = 0
		
		# Min Cell Voltage Module #1
		self.SunSp[112] = Module_Index
		
		# Average Cell Voltage #1
		self.SunSp[113] = (total_voltage/20)
		
		# Total DC Current #1
		self.SunSp[114] = (current)
		
		# Max Charge Current #1
		self.SunSp[115] = (current)
		
		# Max Discharge Current #1
		self.SunSp[116] = (current)
		
		# Total Power #1
		self.SunSp[117] = (current)*(total_voltage)
		
		# Inverter State Request #1
		self.SunSp[118] = 0
		
		# Battery Power Request #1
		self.SunSp[119] = 0xffff
		
		# Set Operation #1
		self.SunSp[120] = 1
		
		# Set Inverter State #1
		self.SunSp[121] = 2
		
		# AHRtg_SF #1
		self.SunSp[122] = 100
		
		# WHRtg_SF #1
		self.SunSp[123] = 100
		
		# WChaDisChaMax_SF #1
		self.SunSp[124] = 100
		
		# DisChaRte_SF #1
		self.SunSp[125] = 100
		
		# SoC_SF #1
		self.SunSp[126] = 100
		
		# DoD_SF #1
		self.SunSp[127] = 100
		
		# SoH_SF #1
		self.SunSp[128] = 100
		
		# V_SF #1
		self.SunSp[129] = 100
		
		# CellV_SF #1
		self.SunSp[130] = 100
		
		# A_SF #1
		self.SunSp[131] = 100
		
		# AMax_SF #1
		self.SunSp[132] = 100
		
		# W_SF #1
		self.SunSp[133] = 100
		
		#************************************#
		#SunSpec Model Lithium-Ion Module Model #805 #
		self.SunSp[134] = 805
		self.SunSp[135] = 94
		
		# String Index #1
		self.SunSp[136] = 0
		
		# Module Index #1
		self.SunSp[137] = Module_Index
		
		# Module Cell Count #1
		self.SunSp[138] = 20
		
		# Module SoC #1
		self.SunSp[139] = SOC
		
		# Depth of Discharge #1
		self.SunSp[140] = (100-SOC)
		
		# Module SoH #1 ***
		self.SunSp[141] = SOH
		
		# Cycle Count #2 ***
		self.SunSp[142] = 0xffff
		self.SunSp[143] = 1
		
		# Module Voltage #1
		self.SunSp[144] = (total_voltage)
		# Max Cell Voltage #1 ***
		self.SunSp[145] = max_vol
		# Max Cell Voltage Cell #1 ***
		self.SunSp[146] = max_vol_num
		# Min Cell Voltage #1 ***
		self.SunSp[147] = min_vol
		# Min Cell Voltage Cell #1 ***
		self.SunSp[148] = min_vol_num
		# Average Cell Voltage #1
		self.SunSp[149] = (total_voltage/20)
		
		# Max Cell Temperature #1
		self.SunSp[150] = (temperature)
		# Max Cell Temperature Cell #1
		self.SunSp[151] = 0xffff
		# Min Cell Temperature #1
		self.SunSp[152] = (temperature)
		# Min Cell Temperature Cell #1
		self.SunSp[153] = 0xffff
		# Average Cell Voltage #1
		self.SunSp[154] = (temperature)
		
		# Balanced Cell Count #1
		self.SunSp[155] = 20
		
		# Serial Number #16
		self.SunSp[156] = 'N' << 8 | 'T'
		self.SunSp[157] = 'U' << 8 | 'T'
		self.SunSp[158] = '0' << 8 | '0'
		self.SunSp[159] = '0' << 8 | '1'
		
		# SoC_SF #1
		self.SunSp[172] = 100
		# SoH_SF #1
		self.SunSp[173] = 100
		# DoD_SF #1
		self.SunSp[174] = 100
		# V_SF #1
		self.SunSp[175] = 100
		# CellV_SF #1
		self.SunSp[176] = 100
		# Tmp_SF#1
		self.SunSp[177] = 100
		
		i = 0
		for n in range(20):
		    # Cell 1~20 #3
		    self.SunSp[178+i] = voltage[n]
		    self.SunSp[179+i] = (temperature)
		    self.SunSp[180+i] = state
		    i = i+3
