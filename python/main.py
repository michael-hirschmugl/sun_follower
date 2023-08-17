# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import serial
import time
import matplotlib.pyplot as plt


def print_hi():
    # Use a breakpoint in the code line below to debug your script.
    print("Read sun follower data:")  # Press Ctrl+F8 to toggle the breakpoint.


def read_x(serInst):
    try:
        buffer = "x" + "\n"
        serInst.write(str.encode(buffer))
        res = serInst.readline()
        decoded_res = res.decode("ascii").strip()
        numeric_chars = [c for c in decoded_res if c.isdigit() or c == '-']
        numeric_str = ''.join(numeric_chars)
        try:
            result_integer = int(numeric_str)
        except:
            return 0;
    except serial.SerialException:
        print("No Connection")
    return result_integer


def read_y(serInst):
    try:
        buffer = "y" + "\n"
        serInst.write(str.encode(buffer))
        res = serInst.readline()
        decoded_res = res.decode("ascii").strip()
        numeric_chars = [c for c in decoded_res if c.isdigit() or c == '-']
        numeric_str = ''.join(numeric_chars)
        try:
            result_integer = int(numeric_str)
        except:
            return 0;
    except serial.SerialException:
        print("No Connection")
    return result_integer


def read_z(serInst):
    try:
        buffer = "z" + "\n"
        serInst.write(str.encode(buffer))
        res = serInst.readline()
        decoded_res = res.decode("ascii").strip()
        numeric_chars = [c for c in decoded_res if c.isdigit() or c == '-']
        numeric_str = ''.join(numeric_chars)
        try:
            result_integer = int(numeric_str)
        except:
            return 0;
    except serial.SerialException:
        print("No Connection")
    return result_integer


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi()
    ser = serial.Serial()
    ser.port = "COM11"
    ser.baudrate = 38400
    ser.open()
    if (ser.isOpen()):
        print("Connected!")
    else:
        print("Not Connected!")

    x_values = []
    y_values = []
    z_values = []

    count = 0

    while (count < 1000):
        x_values.append(read_x(ser))
        time.sleep(0.1)
        y_values.append(read_y(ser))
        time.sleep(0.1)
        z_values.append(read_z(ser))
        time.sleep(0.1)
        print(count)
        count = count + 1

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x_values, y_values, z_values, c='b', marker='o')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
