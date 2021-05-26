import dynamixel_sdk as dxl
import serial.rs485
from tqdm import tqdm

ph = dxl.PacketHandler(2.0)

tests = {'/dev/ttySC0': [51, 52, 41, 42, 43, 44, 31, 32, 33, 34],
         '/dev/ttySC1': [11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26] }

for port_name in tests:
	port = dxl.PortHandler(port_name)
	port.openPort()
	port.setBaudRate(2000000)
	port.ser.rs485_mode = serial.rs485.RS485Settings()

	gsr = dxl.GroupSyncRead(port, ph, 126, 10)

	for dxl_id in tests[port_name]:
		result = gsr.addParam(dxl_id)
		if result != True:
			print("[ID:%03d] groupSyncRead addparam failed" % dxl_id)
			quit()
	errs = 0
	for _ in tqdm(range(5000)):

		dxl_comm_result = gsr.txRxPacket()
		if dxl_comm_result != dxl.COMM_SUCCESS:
			print("%s" % ph.getTxRxResult(dxl_comm_result))
			errs += 1

		for dxl_id in tests[port_name]:
			result = gsr.isAvailable(dxl_id, 126, 2)
			#if result != True:
			#	print(f'ID:{dxl_id} no load')
			result = gsr.isAvailable(dxl_id, 128, 4)
			#if result != True:
                        #        print(f'ID:{dxl_id} no velocity')
			result = gsr.isAvailable(dxl_id, 132, 4)
			#if result != True:
                        #        print(f'ID:{dxl_id} no load')

	print(f'port: {port_name} Total errors: {errs}')

	port.closePort()
