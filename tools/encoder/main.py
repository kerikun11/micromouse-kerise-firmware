# %% import
import math
import serial
import datetime
import os
import numpy as np
import matplotlib.pyplot as plt
import argparse
from matplotlib.ticker import ScalarFormatter
# from scipy.ndimage import gaussian_filter1d

# %% serial import


def serial_import(filename, serial_port, serial_baudrate):
    with serial.Serial(serial_port, serial_baudrate, timeout=10) as ser:
        ser.flush()
        print(f'serial port {serial_port} ({serial_baudrate}) listening...')
        firstline = ser.readline()
        if not firstline:
            print('serial import timeout :(')
            exit(1)
        ser.timeout = 0.1  # shorten timeout after first line
        lines = ser.readlines()
        lines.insert(0, firstline)
        # save to file
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, 'w') as f:
            for line in lines:
                f.write(line.decode())


datetime_string = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
filename = 'data/' + datetime_string + '.csv'
# serial_import(filename, '/dev/ttyUSB0', 2000000)


# %% file import
filename = 'v5/trap.csv'

# raw = np.loadtxt(filename, delimiter=',')
raw = np.loadtxt(filename, delimiter='\t')
print(raw.shape)

# %% plot

ch = 1
N = 2**14

# data = raw[:, ch]
data = raw[raw.shape[0]//3:-raw.shape[0]//3, ch+16]

# plt.figure()
# plt.plot(data)
# y = gaussian_filter1d(data, 8)
# plt.plot(y)

# v_int = [0]
# for v in data:
#     v_int.append(v_int[-1]+v)

v_int = data

v_int_rem = [v % N for v in v_int]
# plt.figure()
# plt.plot(v_int_rem)

v_rem_diff = []
for v1, v2 in zip(v_int_rem, v_int_rem[1:]):
    v_rem_diff.append(v2-v1)
# plt.figure()
# plt.plot(v_rem_diff)


boundary = np.abs(v_rem_diff) > N // 2
boundary = np.where(boundary)[0]

parts = []
for i1, i2 in zip(boundary, boundary[1:]):
    parts.append(v_int_rem[i1+1:i2+1])

if ch == 0:
    # v5
    # k = 0.95e-2
    # p = 0.1e-1
    # v3
    k = 0.9e-2
    p = 0.9e-1
else:
    # k = 4.9e-2
    # p = 7.7e-1
    k = 0.8e-2
    p = 9.9e-1


def char_func(x):
    return x + k * N * math.sin(2*math.pi*(x/N-p))


def inverse_func(x):
    return x + k * N * math.sin(2*math.pi*(x/N-p+0.5))


plt.figure(figsize=(12, 12))
plt.gca().set_aspect('equal')
plt.grid()
for y in parts:
    x = np.linspace(y[0], y[-1], len(y))
    plt.plot(x, y, color='pink')
    y = [inverse_func(v)-inverse_func(0) for v in y]
    plt.plot(x, y, color='lightgreen')
plt.plot([0, N], [0, N], 'b')

x = np.linspace(0, N, N+1)
y = [char_func(xx) for xx in x]
y -= y[0]
# plt.plot(x, y, 'r')

y = [inverse_func(xx) for xx in x]
y -= y[0]
# plt.plot(x, y, 'g')

# plt.savefig('enc.png')

plt.figure(figsize=(12, 12))
# plt.gca().set_aspect('equal')
plt.grid()
for y in parts:
    x = np.linspace(y[0], y[-1], len(y))
    plt.plot(x, y-x, color='pink')
    y = [inverse_func(v)-inverse_func(0) for v in y]
    plt.plot(x, y-x, color='lightgreen')

print(inverse_func(0))
