#!/usr/bin/env python3
import serial
import time
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.2)


def tx(cmd, val):
    frame = f"#{cmd}:{val:.2f};;\r\n"      # *** always two decimals ***
    ser.write(frame.encode())
    print(">>>", frame.strip())
    time.sleep(0.05)
    while (l := ser.readline()):
        print("<<<", l.decode(errors='ignore').strip())


tx('speed', 30)      # speed 30.00
tx('speed', 10)      # steer 10.00
