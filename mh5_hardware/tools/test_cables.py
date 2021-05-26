import dynamixel_sdk as dxl
import serial.rs485

ph = dxl.PacketHandler(2.0)

tests = {'/dev/ttySC0': [51, 52, 41, 42, 43, 44, 31, 32, 33, 34],
         '/dev/ttySC1': [11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26] }

for port_name in tests:
	port = dxl.PortHandler(port_name)
	port.openPort()
	port.setBaudRate(2000000)
	port.ser.rs485_mode = serial.rs485.RS485Settings()

	for dxl_id in tests[port_name]:
		errs = 0
		for _ in range(10000):
			data, result, err = ph.readTxRx(port, dxl_id, 126, 10)
			if result or err:
				errs += 1
		print(f'ID: {dxl_id} Total errors: {errs}')

	port.closePort()
