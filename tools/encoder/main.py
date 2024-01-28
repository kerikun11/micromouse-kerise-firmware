import argparse
import datetime
import os
import serial
import pandas as pd
from serial.tools import list_ports
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


def process(basename, args):
    raw = pd.read_csv(basename+".csv", comment="#", delimiter="\t")
    # print(raw)
    timestamp = raw['timestamp_us']
    plt.figure()
    plt.plot(timestamp.diff(), linestyle='None', marker='.')
    plt.grid()
    plt.xlabel('Sample')
    plt.ylabel('Timestamp Diff [us]')
    plt.title('Timestamp Diff [us]')
    plt.tight_layout()
    # plt.savefig(f'{basename}_timestamp.svg')
    plt.savefig(f'{basename}_timestamp.png')

    N_bits = 14
    N = 2 ** N_bits

    # encoder fitting function
    def char_func_offset(x, a, b):
        return a * np.sin(2 * np.pi * (x / N + b))

    def char_func(x, a, b):
        return x + char_func_offset(x, a, b)

    for ch in [0, 1]:
        # for ch in [0]:
        pulses = raw[f'enc_pulses_{ch}']
        pulses_raw = pulses % N

        # fitting
        initial_guess = [N/2, 0.5]
        pulses_ideal = np.linspace(
            pulses.iloc[0], pulses.iloc[-1], len(pulses.index))
        bounds = ([0, 0], [np.inf, 1])
        params, covariance = curve_fit(
            char_func, pulses, pulses_ideal, p0=initial_guess, bounds=bounds)
        a, b = params
        # print(params, covariance)
        print(f"ch{ch} gain: {a}\tphase: {b}")

        pulses_fixed = char_func(pulses, a, b)
        offset = char_func_offset(pulses, a, b)
        with open(f'{basename}_result_ch{ch}.txt', 'w') as f:
            f.write(f"y = a sin(2 pi (x + b) / N)\n")
            f.write(f"a = {a}\n")
            f.write(f"b = {b}\n")
            f.write(f"N = {N}\n")

        plt.figure(figsize=(6, 8))
        plt.gcf().canvas.manager.set_window_title('Encoder Eccentricity Correction')

        plt.subplot(3, 1, 1)
        plt.plot(pulses_raw)
        plt.ylim([0, N])
        plt.yticks(np.linspace(0, N, 5))
        plt.grid(True)
        plt.ylabel(f'Encoder Raw Value ({N_bits}-bit)')
        plt.title('Encoder Raw Values when Wheel Spinning Freely')

        plt.subplot(3, 1, 2)
        plt.plot(pulses.diff())
        plt.plot(pulses.diff().mean() - offset.diff())
        ylim = plt.ylim()
        plt.grid(True)
        plt.ylabel('Differential of Encoder Values')
        plt.title('Fitting Result of Differential of Encoder Values with Sine')
        # plt.legend(['Differential of Encoder Values', 'Fitting Result with Sine'])

        plt.subplot(3, 1, 3)
        plt.plot(pulses_fixed.diff())
        plt.ylim(ylim)
        plt.grid(True)
        plt.xlabel('Time [ms]')
        plt.ylabel('Differential of Encoder Values')
        plt.title('Correction Result of Differential of Encoder Values')

        plt.tight_layout()
        # plt.savefig(f'{basename}_ch{ch}.svg')
        plt.savefig(f'{basename}_ch{ch}.png')


def find_serial_port():
    ports = list_ports.comports()
    if len(ports) == 0:
        raise RuntimeError("Error: serial device not found")
    elif len(ports) == 1:
        return ports[0].device
    else:
        print("port list:")
        for i, port in enumerate(ports):
            print(f"  - {i}: {port.description}")
        print("please input number of target port >> ", end="")
        port_num = int(input())
        return ports[port_num].device


def import_data_from_serial(filename, serial_port, serial_baudrate):
    first_timeout = 60
    with serial.Serial(serial_port, serial_baudrate, timeout=first_timeout) as ser:
        ser.reset_input_buffer()  # flush
        print(f"port: {ser.name} baudrate: {ser.baudrate} listening ...")
        line = ser.readline()
        if not line:
            raise RuntimeError("serial timeout")
        print("serial import started ...")
        ser.timeout = 0.2  # shorten timeout after first line
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, "w", encoding="utf-8") as f:
            while line:
                f.write(line.decode())
                line = ser.readline()
        print("serial import finished")
        print("filename: ", filename)


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--file", "-f", help="csv data file", default=None)
    parser.add_argument("--dir", "-d", help="save dir", default="./data")
    parser.add_argument("--skip_show", "-s",
                        help="skip to show", action="store_true")
    parser.add_argument("--port", "-p", help="serial port", default=None)
    parser.add_argument(
        "--baud", "-b", help="serial baudrate", default=2_000_000)
    args = parser.parse_args()

    # select csv input
    basename = ""
    if args.file:
        basename = os.path.splitext(args.file)[0]  # remove extension
    else:
        datetime_string = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        basename = f"{args.dir}/{datetime_string}/{datetime_string}"
        filepath_csv = basename + ".csv"
        serial_port = args.port if args.port else find_serial_port()
        import_data_from_serial(filepath_csv, serial_port, args.baud)

    # process csv
    process(basename, args)

    # show
    if not args.skip_show:
        plt.show()


if __name__ == "__main__":
    main()
