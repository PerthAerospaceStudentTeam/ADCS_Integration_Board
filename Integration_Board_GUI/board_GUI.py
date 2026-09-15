from time import sleep

import serial.tools.list_ports

# Searching through current serial ports for the ADCS Integration Board device
ports = serial.tools.list_ports.comports()
for p in ports:
    if "STMicroelectronics STLink Virtual COM Port" in p.description:
        print(f"Found the device: {p.description}\nUsing the port: {p.device}\n")
        board_port = p.device
    else:
        board_port = None

    sleep(2)  # So the port being used can be read

# Start of data processing loop
with open("sensor_log.csv", "w") as log_file:  # Implement adding date/time to log name?
    with serial.Serial(board_port, 115200, timeout=1) as board_serial:
        while(1):
            if board_serial.in_waiting > 0:
                raw_data = board_serial.readline()
                data = raw_data.decode("utf-8")
                log_file.write(data)
                print(data, end="")