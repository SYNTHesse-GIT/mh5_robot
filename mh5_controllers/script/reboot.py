#!/usr/bin/env python3

import dynamixel_sdk as dyn
from serial import rs485

ports = ['/dev/ttySC0', '/dev/ttySC1']

for port_name in ports:
    port = dyn.PortHandler(port_name)
    port.openPort()
    port.setBaudRate(2000000)
    port.ser.rs485_mode = rs485.RS485Settings()
    ph = dyn.PacketHandler(2.0)
    print(f'\nPort: {port_name}')
    for dxl_id in range(60):
        model_number, result, error = ph.ping(port, dxl_id)
        if result == dyn.COMM_SUCCESS:
            hwerr, _, _ = ph.read1ByteTxRx(port, dxl_id, 70)
            print(f'ID: {dxl_id}, Model: {model_number}, Error: {error}, HWErr: {hwerr}')
            if hwerr != 0:
                result, error = ph.reboot(port, dxl_id)
                print(f'Rebooted {dxl_id}, Result: {result}, Error: {error}')

    port.closePort()
