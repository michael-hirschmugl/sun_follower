# This script demonstrates how to read three different values (X, Y, Z) from a
# serial device (referred to as the "sun follower"). The script collects 1000
# data points for each axis, then plots them in a 3D scatter plot using matplotlib.

import serial
import time
import matplotlib.pyplot as plt

def print_hi():
    """
    Print an introductory message.
    """
    print("Reading sun follower data...")

def read_x(serInst):
    """
    Request the X coordinate from the serial device, read the response, 
    and parse it into an integer. Returns 0 if reading or conversion fails.
    
    :param serInst: An open serial.Serial instance
    :return: Integer value of the X coordinate or 0 if an error occurs
    """
    try:
        buffer = "x\n"
        serInst.write(str.encode(buffer))
        res = serInst.readline()
        decoded_res = res.decode("ascii").strip()
        numeric_chars = [c for c in decoded_res if c.isdigit() or c == '-']
        numeric_str = ''.join(numeric_chars)
        try:
            result_integer = int(numeric_str)
        except:
            return 0
    except serial.SerialException:
        print("No connection to the serial device.")
        return 0
    return result_integer

def read_y(serInst):
    """
    Request the Y coordinate from the serial device, read the response, 
    and parse it into an integer. Returns 0 if reading or conversion fails.
    
    :param serInst: An open serial.Serial instance
    :return: Integer value of the Y coordinate or 0 if an error occurs
    """
    try:
        buffer = "y\n"
        serInst.write(str.encode(buffer))
        res = serInst.readline()
        decoded_res = res.decode("ascii").strip()
        numeric_chars = [c for c in decoded_res if c.isdigit() or c == '-']
        numeric_str = ''.join(numeric_chars)
        try:
            result_integer = int(numeric_str)
        except:
            return 0
    except serial.SerialException:
        print("No connection to the serial device.")
        return 0
    return result_integer

def read_z(serInst):
    """
    Request the Z coordinate from the serial device, read the response, 
    and parse it into an integer. Returns 0 if reading or conversion fails.
    
    :param serInst: An open serial.Serial instance
    :return: Integer value of the Z coordinate or 0 if an error occurs
    """
    try:
        buffer = "z\n"
        serInst.write(str.encode(buffer))
        res = serInst.readline()
        decoded_res = res.decode("ascii").strip()
        numeric_chars = [c for c in decoded_res if c.isdigit() or c == '-']
        numeric_str = ''.join(numeric_chars)
        try:
            result_integer = int(numeric_str)
        except:
            return 0
    except serial.SerialException:
        print("No connection to the serial device.")
        return 0
    return result_integer

if __name__ == '__main__':
    # Print an initial message
    print_hi()

    # Configure and open the serial port.
    # Make sure the COM port and baud rate match your device settings.
    ser = serial.Serial()
    ser.port = "COM11"
    ser.baudrate = 38400
    ser.open()

    # Check if the serial connection was successfully opened.
    if ser.isOpen():
        print("Connected to the serial device.")
    else:
        print("Failed to connect to the serial device.")

    # Prepare lists to store 1000 readings for each axis.
    x_values = []
    y_values = []
    z_values = []

    count = 0

    # Collect 1000 data samples for each axis, adding a short delay between reads.
    while count < 1000:
        x_values.append(read_x(ser))
        time.sleep(0.1)
        y_values.append(read_y(ser))
        time.sleep(0.1)
        z_values.append(read_z(ser))
        time.sleep(0.1)
        print(count)  # Print current iteration count for reference.
        count += 1

    # Create a 3D scatter plot to visualize the collected data.
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x_values, y_values, z_values, c='b', marker='o')

    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    # Display the plot.
    plt.show()
