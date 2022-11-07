import numpy as np

SunSp = np.zeros(1000, dtype='uint16')
Slave_Address = 1
Module_Index = 1


def SunS_mapping(current, voltage, temperature, state, error_code, SOC, SOH, total_voltage, max_vol, min_vol, max_vol_num, min_vol_num):

	# ************************************#
	# SunSpec Identifier
	SunSp[0] = 'S' << 8 | 'u'
    	SunSp[1] = 'n' << 8 | 'S'

	#************************************#
	#SunSpec Model Common #1 #
	SunSp[2] = 1
	SunSp[3] = 66
	
	# Manufacturer #16
	SunSp[4] = 'N' << 8 | 'T'
	SunSp[5] = 'U' << 8 | 'T'
	SunSp[6] = 'L' << 8 | 'A'
	SunSp[7] = 'B' << 8 | '3'
	SunSp[8] = '1' << 8 | '4'
	
	# Model #16
	SunSp[20] = 'L' << 8 | 'i'
	SunSp[21] = 't' << 8 | 'h'
	SunSp[22] = 'i' << 8 | 'u'
	SunSp[23] = 'm' << 8 | '-'
	SunSp[24] = 'I' << 8 | 'o'
	SunSp[25] = 'n' << 8 | ' '
	SunSp[26] = 'B' << 8 | 'a'
	SunSp[27] = 't' << 8 | 't'
	SunSp[28] = 'e' << 8 | 'r'
	SunSp[29] = 'y'
	
	# Options #8
	SunSp[36] = 'X' << 8 | 'i'
	SunSp[37] = 'a' << 8 | 'n'
	SunSp[38] = 'g' << 8 | '-'
	SunSp[39] = 'Y' << 8 | 'a'
	SunSp[40] = 'n' << 8 | 'g'
	
	# Version #8
	SunSp[44] = 'V' << 8 | 'e'
	SunSp[45] = 'r' << 8 | ' '
	SunSp[46] = '1' << 8 | '.'
	SunSp[47] = '0'
	
	# Serial Number #16
	SunSp[52] = 'N' << 8 | 'T'
	SunSp[53] = 'U' << 8 | 'T'
	SunSp[54] = '0' << 8 | '0'
	SunSp[55] = '0' << 8 | '1'
	
	# Device Address #1
	SunSp[68] = Slave_Address
	
	# Pad #1
	SunSp[69] = 0xffff
	
	#************************************#
	#SunSpec Model Battery Base Model #802 #
	SunSp[70] = 802
	SunSp[71] = 62
	
	# Nameplate Charge Capacity #1
	SunSp[72] = 25
	
	# Nameplate Energy Capacity #1
	SunSp[73] = 1365
	
	# Nameplate Max Charge Rate #1
	SunSp[74] = 90
	
	# Namplate Max Discharge Rate #1
	SunSp[75] = 90
	
	# Self Discharge Rate #1
	SunSp[76] = 10
	
	# Nameplate Max SoC #1
	SunSp[77] = 80
	
	# Nameplate Min SoC #1
	SunSp[78] = 20
	
	# Max Reserve Percent #1
	SunSp[79] = 80
	
	# Min Reserve Percent #1
	SunSp[80] = 20
	
	# State of Charge #1
	SunSp[81] = SOC
	
	# Depth of Discharge #1
	SunSp[82] = (100-SOC)
	
	# State of Health #1
	SunSp[83] = 100
	
	# Cycle Count #2
	SunSp[84] = 0xffff
	SunSp[85] = 1
	
	# Charge Status #1
	SunSp[86] = 6
	
	# Control Mode #1
	SunSp[87] = 1
	
	# Battery Heartbeat #1
	SunSp[88] = 1
	
	# Controller Heartbeat #1
	SunSp[89] = 1
	
	# Alarm Reset #1
	SunSp[90] = 0
	
	# Battery Type #1
	SunSp[91] = 4
	
	# State of the Battery Bank #1
	SunSp[92] = 4
	
	# Vendor Battery Bank State #1
	SunSp[93] = 4
	
	# Warranty Date #2
	SunSp[94] = 2022
	SunSp[95] = 216
	
	# Battery Event 1 Bitfield #2
	SunSp[96] = 0xffff
	SunSp[97] = 0xffff
	# Battery Event 2 Bitfield #2
	SunSp[98] = 0xffff
	SunSp[99] = 0xffff
	# Vendor Event Bitfield 1  #2
	SunSp[100] = 0xffff
	SunSp[101] = 0xffff
	# Vendor Event Bitfield 2  #2
	SunSp[102] = 0xffff
	SunSp[103] = 0xffff
	
	# External Battery Voltage #1
	SunSp[104] = (total_voltage)
	
	# Max Battery Voltage #1
	SunSp[105] = (total_voltage)
	
	# Min Battery Voltage #1
	SunSp[106] = (total_voltage)
	
	# Max Cell Voltage #1
	SunSp[107] = max_vol
	
	# Max Cell Voltage String #1
	SunSp[108] = 0
	
	# Max Cell Voltage Module #1
	SunSp[109] = Module_Index
	
	# Min Cell Voltage #1
	SunSp[110] = min_vol
	
	# Min Cell Voltage String #1
	SunSp[111] = 0
	
	# Min Cell Voltage Module #1
	SunSp[112] = Module_Index
	
	# Average Cell Voltage #1
	SunSp[113] = (total_voltage/13)
	
	# Total DC Current #1
	SunSp[114] = (current)
	
	# Max Charge Current #1
	SunSp[115] = (current)
	
	# Max Discharge Current #1
	SunSp[116] = (current)
	
	# Total Power #1
	SunSp[117] = (current)*(total_voltage)
	
	# Inverter State Request #1
	SunSp[118] = 0
	
	# Battery Power Request #1
	SunSp[119] = 0xffff
	
	# Set Operation #1
	SunSp[120] = 1
	
	# Set Inverter State #1
	SunSp[121] = 2
	
	# AHRtg_SF #1
	SunSp[122] = 100
	
	# WHRtg_SF #1
	SunSp[123] = 100
	
	# WChaDisChaMax_SF #1
	SunSp[124] = 100
	
	# DisChaRte_SF #1
	SunSp[125] = 100
	
	# SoC_SF #1
	SunSp[126] = 100
	
	# DoD_SF #1
	SunSp[127] = 100
	
	# SoH_SF #1
	SunSp[128] = 100
	
	# V_SF #1
	SunSp[129] = 100
	
	# CellV_SF #1
	SunSp[130] = 100
	
	# A_SF #1
	SunSp[131] = 100
	
	# AMax_SF #1
	SunSp[132] = 100
	
	# W_SF #1
	SunSp[133] = 100
	
	#************************************#
	#SunSpec Model Lithium-Ion Module Model #805 #
	SunSp[134] = 805
	SunSp[135] = 94
	
	# String Index #1
	SunSp[136] = 0
	
	# Module Index #1
	SunSp[137] = Module_Index
	
	# Module Cell Count #1
	SunSp[138] = 13
	
	# Module SoC #1
	SunSp[139] = SOC
	
	# Depth of Discharge #1
	SunSp[140] = (100-SOC)
	
	# Module SoH #1 ***
	SunSp[141] = SOH
	
	# Cycle Count #2 ***
	SunSp[142] = 0xffff
	SunSp[143] = 1
	
	# Module Voltage #1
	SunSp[144] = (total_voltage)
	# Max Cell Voltage #1 ***
	SunSp[145] = max_vol
	# Max Cell Voltage Cell #1 ***
	SunSp[146] = max_vol_num
	# Min Cell Voltage #1 ***
	SunSp[147] = min_vol
	# Min Cell Voltage Cell #1 ***
	SunSp[148] = min_vol_num
	# Average Cell Voltage #1
	SunSp[149] = (total_voltage/13)
	
	# Max Cell Temperature #1
	SunSp[150] = (temperature)
	# Max Cell Temperature Cell #1
	SunSp[151] = 0xffff
	# Min Cell Temperature #1
	SunSp[152] = (temperature)
	# Min Cell Temperature Cell #1
	SunSp[153] = 0xffff
	# Average Cell Voltage #1
	SunSp[154] = (temperature)
	
	# Balanced Cell Count #1
	SunSp[155] = 13
	
	# Serial Number #16
	SunSp[156] = 'N' << 8 | 'T'
	SunSp[157] = 'U' << 8 | 'T'
	SunSp[158] = '0' << 8 | '0'
	SunSp[159] = '0' << 8 | '1'
	
	# SoC_SF #1
	SunSp[172] = 100
	# SoH_SF #1
	SunSp[173] = 100
	# DoD_SF #1
	SunSp[174] = 100
	# V_SF #1
	SunSp[175] = 100
	# CellV_SF #1
	SunSp[176] = 100
	# Tmp_SF#1
	SunSp[177] = 100
	
	i = 0
	for n in range(13):
	    # Cell 1~13 #3
	    SunSp[178+i] = voltage[n]
	    SunSp[179+i] = (temperature)
	    SunSp[180+i] = state
	    i = i+3
