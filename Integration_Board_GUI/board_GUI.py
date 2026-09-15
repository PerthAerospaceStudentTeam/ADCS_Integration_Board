import serial.tools.list_ports
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# Searching through current serial ports for the ADCS Integration Board device
ports = serial.tools.list_ports.comports()
for p in ports:
    if "STMicroelectronics STLink Virtual COM Port" in p.description:
        print(f"Found the device: {p.description}\nUsing the port: {p.device}\n")
        board_port = p.device
    else:
        board_port = None

# Creation of live sensor data GUI elements ------------------------------------
PLOT_VALUES = 500  # Number of values to store and plot

# Creating plot widget for sensor data graphs
app = pg.mkQApp("Sensor Data GUI")
pw = pg.GraphicsLayoutWidget(show=True, title="Sensor Data GUI")
pw.setWindowTitle("Sensor Data GUI")
pw.resize(1000, 1000)

# Accelerometer data plot
acc_plt = pw.addPlot(title="Accelerometer (mg)", row=0, col=0)
acc_plt.addLegend()
acc_1 = acc_plt.plot(pen='r', name="X")
acc_2 = acc_plt.plot(pen='g', name="Y")
acc_3 = acc_plt.plot(pen='b', name="Z")

# Gyroscope data plot
gyro_plt = pw.addPlot(title="Gyroscope (mdps)", row=0, col=1)
gyro_plt.addLegend()
gyro_1 = gyro_plt.plot(pen='r', name="X")
gyro_2 = gyro_plt.plot(pen='g', name="Y")
gyro_3 = gyro_plt.plot(pen='b', name="Z")

# Magnetometer data plot
mag_plt = pw.addPlot(title="Magnetometer (mgauss)", row=1, col=0)
mag_plt.addLegend()
mag_1 = mag_plt.plot(pen='r', name="X")
mag_2 = mag_plt.plot(pen='g', name="Y")
mag_3 = mag_plt.plot(pen='b', name="Z")

# Sun sensor data plot
sun_plt = pw.addPlot(title="Sun sensors", row=1, col=1)
sun_plt.addLegend()
sun_1 = sun_plt.plot(pen='r', name="Z-")
sun_2 = sun_plt.plot(pen='g', name="Z+")
sun_3 = sun_plt.plot(pen='b', name="X+")
sun_4 = sun_plt.plot(pen='m', name="Y+")
sun_5 = sun_plt.plot(pen='y', name="X-")
sun_6 = sun_plt.plot(pen='c', name="Y-")

# Rolling data arrays for plotting sensor data
acc_arr = np.zeros((3, PLOT_VALUES))
gyro_arr = np.zeros((3, PLOT_VALUES))
mag_arr = np.zeros((3, PLOT_VALUES))
sun_arr = np.zeros((6, PLOT_VALUES))
tick_arr = np.zeros(PLOT_VALUES)


# The function called every time the GUI's QTimer creates a signal
def update():
    global log_file, board_serial
    if board_serial.in_waiting > 0:
        # Reading sensor data from serial port and writing to a log file
        raw_data = board_serial.readline()
        data = raw_data.decode("utf-8")
        log_file.write(data)

        # Converting data string to correct data types
        data = data.strip("\n").split(",")
        data[0] = int(data[0])
        for i in range(2, len(data)):
            data[i] = float(data[i])

        # Plotting sensor data according to data label
        if data[1] == "ACC":
            for i in range(PLOT_VALUES - 1):
                tick_arr[i] = tick_arr[i + 1]
                for a in range(len(acc_arr)):
                    acc_arr[a][i] = acc_arr[a][i + 1]

            tick_arr[PLOT_VALUES - 1] = data[0]
            acc_arr[0][PLOT_VALUES - 1] = data[2]
            acc_arr[1][PLOT_VALUES - 1] = data[3]
            acc_arr[2][PLOT_VALUES - 1] = data[4]

            acc_1.setData(x=tick_arr, y=acc_arr[0])
            acc_2.setData(x=tick_arr, y=acc_arr[1])
            acc_3.setData(x=tick_arr, y=acc_arr[2])

        elif data[1] == "GRO":
            for i in range(PLOT_VALUES - 1):
                tick_arr[i] = tick_arr[i + 1]
                for a in range(len(gyro_arr)):
                    gyro_arr[a][i] = gyro_arr[a][i + 1]

            tick_arr[PLOT_VALUES - 1] = data[0]
            gyro_arr[0][PLOT_VALUES - 1] = data[2]
            gyro_arr[1][PLOT_VALUES - 1] = data[3]
            gyro_arr[2][PLOT_VALUES - 1] = data[4]

            gyro_1.setData(x=tick_arr, y=gyro_arr[0])
            gyro_2.setData(x=tick_arr, y=gyro_arr[1])
            gyro_3.setData(x=tick_arr, y=gyro_arr[2])

        elif data[1] == "MAG":
            for i in range(PLOT_VALUES - 1):
                tick_arr[i] = tick_arr[i + 1]
                for a in range(len(mag_arr)):
                    mag_arr[a][i] = mag_arr[a][i + 1]

            tick_arr[PLOT_VALUES - 1] = data[0]
            mag_arr[0][PLOT_VALUES - 1] = data[2]
            mag_arr[1][PLOT_VALUES - 1] = data[3]
            mag_arr[2][PLOT_VALUES - 1] = data[4]

            mag_1.setData(x=tick_arr, y=mag_arr[0])
            mag_2.setData(x=tick_arr, y=mag_arr[1])
            mag_3.setData(x=tick_arr, y=mag_arr[2])

        elif data[1] == "SUN":
            for i in range(PLOT_VALUES - 1):
                tick_arr[i] = tick_arr[i + 1]
                for a in range(len(sun_arr)):
                    sun_arr[a][i] = sun_arr[a][i + 1]

            tick_arr[PLOT_VALUES - 1] = data[0]
            sun_arr[0][PLOT_VALUES - 1] = data[2]
            sun_arr[1][PLOT_VALUES - 1] = data[3]
            sun_arr[2][PLOT_VALUES - 1] = data[4]
            sun_arr[3][PLOT_VALUES - 1] = data[5]
            sun_arr[4][PLOT_VALUES - 1] = data[6]
            sun_arr[5][PLOT_VALUES - 1] = data[7]

            sun_1.setData(x=tick_arr, y=sun_arr[0])
            sun_2.setData(x=tick_arr, y=sun_arr[1])
            sun_3.setData(x=tick_arr, y=sun_arr[2])
            sun_4.setData(x=tick_arr, y=sun_arr[3])
            sun_5.setData(x=tick_arr, y=sun_arr[4])
            sun_6.setData(x=tick_arr, y=sun_arr[5])

        else:
            print("Unknown Data Type")


# Start of the GUI update loop
with open("sensor_log.csv", "w") as log_file:  # Implement adding date/time to log name?
    with serial.Serial(board_port, 115200, timeout=1) as board_serial:
        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(1)

        if __name__ == "__main__":
            pg.exec()
