#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import argparse


def mm2inch(mm):
    return 0.03937008 * mm


def load_file(csv_file, index_time, index_value, start_time, end_time):

    data_array = np.loadtxt(csv_file, skiprows=1, delimiter=',')
    raw_array_time = 0.000000001 * \
        (data_array[:, index_time] - data_array[0, index_time])
    raw_array_value = data_array[:, index_value]

    target_range = np.logical_and(raw_array_time > start_time, raw_array_time < end_time)
    array_time = raw_array_time[target_range]
    array_value = raw_array_value[target_range]

    return array_time, array_value


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file_raw')
    parser.add_argument('csv_file_lpf')
    parser.add_argument('--start-time', default=0, type=float)
    parser.add_argument('--end-time', default=10, type=float)
    parser.add_argument('--index-time-raw', default=0, type=int)
    parser.add_argument('--index-value-raw', default=3, type=int)
    parser.add_argument('--index-time-lpf', default=0, type=int)
    parser.add_argument('--index-value-lpf', default=3, type=int)
    parser.add_argument('--label-value', default='Acceleration [m/s^2]', type=str)
    parser.add_argument('--title', default='Low Pass Filter for Acceleration', type=str)
    parser.add_argument('--font-size', default=10, type=float)
    parser.add_argument('--figsize-width', default=75, type=int)
    parser.add_argument('--figsize-height', default=50, type=int)
    parser.add_argument('--line-width', default=0.2, type=float)
    args = parser.parse_args()

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['font.size'] = args.font_size

    array_time_raw, array_value_raw = load_file(
            args.csv_file_raw,
            args.index_time_raw,
            args.index_value_raw,
            args.start_time,
            args.end_time
            )

    array_time_lpf, array_value_lpf = load_file(
            args.csv_file_lpf,
            args.index_time_lpf,
            args.index_value_lpf,
            args.start_time,
            args.end_time
            )

    fig, ax = plt.subplots(
            1,
            1,
            figsize=(mm2inch(args.figsize_width), mm2inch(args.figsize_height)),
            tight_layout=True)
    ax.plot(array_time_raw, array_value_raw, lw=args.line_width, color='blue', label='raw')
    ax.plot(array_time_lpf, array_value_lpf, lw=args.line_width, color='red', label='lpf')
    ax.legend()
    ax.set_xlabel('Time [sec]')
    ax.set_ylabel('{}'.format(args.label_value))
    ax.set_title('{}'.format(args.title))
    plt.show()


if __name__ == '__main__':
    main()
