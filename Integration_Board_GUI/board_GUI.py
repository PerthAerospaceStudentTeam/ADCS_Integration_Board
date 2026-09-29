import serial.tools.list_ports
import numpy as np
import datetime
import time

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

# ------------------------------- GUI Components -------------------------------
PLOT_VALUES = 500  # Number of values to store and plot

# Create the main GUI app and widget for plotting basic sensor data
sensor_gui = pg.mkQApp("Sensor Data GUI")
basic_widget = pg.GraphicsLayoutWidget(show=True, title="Basic Sensor Data GUI")
basic_widget.resize(1000, 1000)
basic_widget.setWindowTitle("Basic Sensor Data GUI")

# Create the basic accelerometer data plot
acc_plt = basic_widget.addPlot(title="Accelerometer (mg)", row=0, col=0)
acc_plt.addLegend()

# Create the basic gyroscope data plot
gyro_plt = basic_widget.addPlot(title="Gyroscope (mdps)", row=0, col=1)
gyro_plt.addLegend()

# Create the basic magnetometer data plot
mag_plt = basic_widget.addPlot(title="Magnetometer (mgauss)", row=1, col=0)
mag_plt.addLegend()

# Create the basic sun sensor data plot
sun_plt = basic_widget.addPlot(title="Sun sensors", row=1, col=1)
sun_plt.addLegend()

# Create a dictionary for organising the plots for each sensor axis
plot_dict = {"ACC": [acc_plt.plot(pen='r', name="X"),
                     acc_plt.plot(pen='g', name="Y"),
                     acc_plt.plot(pen='b', name="Z")],

             "GRO": [gyro_plt.plot(pen='r', name="X"),
                     gyro_plt.plot(pen='g', name="Y"),
                     gyro_plt.plot(pen='b', name="Z")],

             "MAG": [mag_plt.plot(pen='r', name="X"),
                     mag_plt.plot(pen='g', name="Y"),
                     mag_plt.plot(pen='b', name="Z")],

             "SUN": [sun_plt.plot(pen='c', name="-Z"),
                     sun_plt.plot(pen='b', name="+Z"),
                     sun_plt.plot(pen='r', name="+X"),
                     sun_plt.plot(pen='g', name="+Y"),
                     sun_plt.plot(pen='m', name="-X"),
                     sun_plt.plot(pen='y', name="-Y")]}

# Create data arrays for storing the data for each sensor axis
# NOTE: arr[0] = a sensor's tick array (x-axis)
data_dict = {"ACC": np.zeros((4, PLOT_VALUES)),
             "GRO": np.zeros((4, PLOT_VALUES)),
             "MAG": np.zeros((4, PLOT_VALUES)),
             "SUN": np.zeros((7, PLOT_VALUES))}


# ---------------------------- GUI Update Function -----------------------------
def update():
    """Update the GUI plots with new sensor data read from a serial port.

    The `update()` function is connected to the QTimer `timer` created below and
    is called everytime `timer` generates a `timeout()` signal.

    Takes and returns no values.
    """
    global serial_buffer

    # Read all data from the serial port into the serial buffer
    serial_data = board_serial.read(board_serial.in_waiting)
    serial_buffer.extend(serial_data)

    # Only read complete lines (terminating with '\n') from the serial buffer
    while b'\n' in serial_buffer:
        update_start = time.perf_counter()

        # Extract and decode the first line from the serial buffer
        serial_line, dl, serial_buffer = serial_buffer.partition(b'\n')
        decoded_data = serial_line.decode("utf-8")

        log_file.write(decoded_data + '\n')

        # Process the decoded data and plot the new sensor data (tick + axis)
        if "|DATA|" in decoded_data:
            new_data = decoded_data[7:].split(",")  # '|DATA| ' = 7 char, so data is from i = 7 onwards

            if new_data[0] in data_dict:
                new_data[1] = int(new_data[1])  # Converts the new data tick
                new_data[2:] = [float(d) for d in new_data[2:]]  # Converts the new axis data

                # Update the sensor's data arrays with the new data
                data_arr = data_dict[new_data[0]]
                data_arr[:, :-1] = data_arr[:, 1:]  # Shuffles down sensor data arrays by 1 place
                data_arr[:, -1] = new_data[1:]  # Adds new sensor data to the end of sensor data arrays

                # Update the sensor's data plots with the new data
                plot_arr = plot_dict[new_data[0]]
                for i in range(len(plot_arr)):
                    plot_arr[i].setData(x=data_arr[0], y=data_arr[i + 1])  # Plots the new sensor data

                print(f"|DEBUG| bytes waiting: {board_serial.in_waiting}")
                print(f"|DEBUG| raw_data: {serial_data}")
                print(f"|DEBUG| serial_data: {serial_data}", end="")
                print(f"|DEBUG| new_data: {new_data}")

        update_end = time.perf_counter()
        print(f"|DEBUG| Update took {(update_end - update_start) * 1000} ms\n")


# -------------------------------- GUI Program ---------------------------------
# Search through available serial ports for the ADCS Integration Board device
ports = serial.tools.list_ports.comports()
for p in ports:
    if "STMicroelectronics STLink Virtual COM Port" in p.description:
        print(f"Found the device: {p.description}\nUsing the port: {p.device}")

        # Create a log file name containing the current date/time
        dt = datetime.datetime.now()
        log_name = f"{dt.day}-{dt.month}-{dt.year}_{dt.hour}{dt.minute}{dt.second}.log"
        print(f"Created the log file: {log_name}")

        serial_buffer = bytearray()
        with open(log_name, "w") as log_file:
            with serial.Serial(p.device, 115200, timeout=0) as board_serial:
                timer = QtCore.QTimer()
                timer.timeout.connect(update)  # Connects the `timer.timeout()` signal to the `update()` function
                timer.start(1)  # Generates a `timeout()` signal every 1 ms

                if __name__ == "__main__":
                    pg.exec()  # Starts the GUI application
