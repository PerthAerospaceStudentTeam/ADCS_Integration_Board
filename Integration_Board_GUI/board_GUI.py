import serial.tools.list_ports
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

# Creating GUI elements --------------------------------------------------------
PLOT_VALUES = 100  # Number of values to store and plot

# Creating plot widget for sensor data graphs
app = pg.mkQApp("Sensor Data GUI")
win = pg.GraphicsLayoutWidget(show=True, title="Sensor Data GUI")
win.resize(1000, 1000)
win.setWindowTitle("Sensor Data GUI")

# Accelerometer data plot
acc_plt = win.addPlot(title="Accelerometer (mg)", row=0, col=0)
acc_plt.addLegend()
acc_1 = acc_plt.plot(pen='r', name="X")
acc_2 = acc_plt.plot(pen='g', name="Y")
acc_3 = acc_plt.plot(pen='b', name="Z")

# Gyroscope data plot
gyro_plt = win.addPlot(title="Gyroscope (mdps)", row=0, col=1)
gyro_plt.addLegend()
gyro_1 = gyro_plt.plot(pen='r', name="X")
gyro_2 = gyro_plt.plot(pen='g', name="Y")
gyro_3 = gyro_plt.plot(pen='b', name="Z")

# Magnetometer data plot
mag_plt = win.addPlot(title="Magnetometer (mgauss)", row=1, col=0)
mag_plt.addLegend()
mag_1 = mag_plt.plot(pen='r', name="X")
mag_2 = mag_plt.plot(pen='g', name="Y")
mag_3 = mag_plt.plot(pen='b', name="Z")

# Sun sensor data plot
sun_plt = win.addPlot(title="Sun sensors", row=1, col=1)
sun_plt.addLegend()
sun_2 = sun_plt.plot(pen='r', name="Z+")
sun_1 = sun_plt.plot(pen='m', name="Z-")
sun_3 = sun_plt.plot(pen='g', name="X+")
sun_5 = sun_plt.plot(pen='c', name="X-")
sun_4 = sun_plt.plot(pen='b', name="Y+")
sun_6 = sun_plt.plot(pen='y', name="Y-")

plot_dict = {"ACC": [acc_1, acc_2, acc_3],
             "GRO": [gyro_1, gyro_2, gyro_3],
             "MAG": [mag_1, mag_2, mag_3],
             "SUN": [sun_1, sun_2, sun_3, sun_4, sun_5, sun_6]}

# Rolling data arrays for plotting sensor data arr[0] = tick values (x-axis values)
acc_arr = np.zeros((4, PLOT_VALUES))
gyro_arr = np.zeros((4, PLOT_VALUES))
mag_arr = np.zeros((4, PLOT_VALUES))
sun_arr = np.zeros((7, PLOT_VALUES))

data_dict = {"ACC": acc_arr,
             "GRO": gyro_arr,
             "MAG": mag_arr,
             "SUN": sun_arr}


def update():  # Called every time the GUI's QTimer creates a signal
    if board_serial.in_waiting > 0:  # To avoid reading from an empty serial port
        # Writing serial data to a .csv file
        raw_data = board_serial.readline()
        data = raw_data.decode("utf-8")
        log_file.write(data)

        # Converting serial data string to the correct data types
        data = data.strip("\n").split(",")
        data[0] = int(data[0])
        for i in range(2, len(data)):
            data[i] = float(data[i])

        # Shuffling down each data array
        data_arr = data_dict[data[1]]
        for i in range(PLOT_VALUES - 1):
            data_arr[0][i] = data_arr[0][i + 1]  # Shuffling down tick data
            for a in range(1, len(data_arr)):
                data_arr[a][i] = data_arr[a][i + 1]  # Shuffling down axis data

        # Appending new data to each data array
        plot_arr = plot_dict[data[1]]
        data_arr[0][PLOT_VALUES - 1] = data[0]  # Appending new tick data
        for a in range(len(data_arr) - 1):
            data_arr[a + 1][PLOT_VALUES - 1] = data[a + 2]  # Appending new axis data
            plot_arr[a].setData(x=data_arr[0], y=data_arr[a + 1])


# Searching through current serial ports for the ADCS Integration Board device
ports = serial.tools.list_ports.comports()
for p in ports:
    if "STMicroelectronics STLink Virtual COM Port" in p.description:
        print(f"Found the device: {p.description}\nUsing the port: {p.device}\n")
        # Start of the GUI update loop
        with open("sensor_log.csv", "w") as log_file:  # Implement adding date/time to log name?
            with serial.Serial(p.device, 115200, timeout=1) as board_serial:
                timer = QtCore.QTimer()
                timer.timeout.connect(update)
                timer.start(1)  # Update the GUI every second

                if __name__ == "__main__":
                    pg.exec()
