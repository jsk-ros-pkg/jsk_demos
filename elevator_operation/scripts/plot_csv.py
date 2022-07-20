#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import argparse


def mm2inch(mm):
    return 0.03937008 * mm


def calc_stat(array):
    min_value = np.min(array)
    max_value = np.max(array)
    ave_value = np.average(array)
    std_value = np.std(array)
    return min_value, max_value, ave_value, std_value


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file')
    parser.add_argument('--start-time', default=0, type=float)
    parser.add_argument('--end-time', default=10, type=float)
    parser.add_argument('--index-time', default=0, type=int)
    parser.add_argument('--index-value', default=3, type=int)
    parser.add_argument('--label-value', default='Air Pressure [Pa]', type=str)
    parser.add_argument('--title', default='Air Pressure during elevator', type=str)
    parser.add_argument('--font-size', default=10, type=float)
    parser.add_argument('--figsize-width', default=75, type=int)
    parser.add_argument('--figsize-height', default=50, type=int)
    parser.add_argument('--line-width', default=0.1, type=float)
    parser.add_argument('--calc-stat-ranges', default='', type=str)
    args = parser.parse_args()

    index_time = args.index_time
    index_value = args.index_value

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['font.size'] = args.font_size

    data_array = np.loadtxt(args.csv_file, skiprows=1, delimiter=',')
    raw_array_time = 0.000000001 * \
        (data_array[:, index_time] - data_array[0, index_time])
    raw_array_value = data_array[:, index_value]

    target_range = np.logical_and(raw_array_time > args.start_time, raw_array_time < args.end_time)
    array_time = raw_array_time[target_range]
    array_value = raw_array_value[target_range]

    calc_stat_ranges = eval(args.calc_stat_ranges)

    fig, ax = plt.subplots(
            1,
            1,
            figsize=(mm2inch(args.figsize_width), mm2inch(args.figsize_height)),
            tight_layout=True)
    ax.plot(array_time, array_value, lw=args.line_width, color='blue')

    for ct_range in calc_stat_ranges:
        target_range = np.logical_and(raw_array_time > ct_range[0], raw_array_time < ct_range[1])
        calc_stat_range_value = raw_array_value[target_range]
        min_value, max_value, ave_value, std_value = calc_stat(calc_stat_range_value)
        # plot line and text for min/max value 
        ax.hlines(min_value, ct_range[0], ct_range[1], linewidth=1, color='red')
        ax.hlines(max_value, ct_range[0], ct_range[1], linewidth=1, color='red')
        ax.text(ct_range[0], (min_value+max_value)/2.0, '{}'.format(int(max_value-min_value)), color='red')
        # plot line and text for average value 
        ax.hlines(ave_value, ct_range[0], ct_range[1], linewidth=1, color='yellow')
        ax.text(ct_range[1], ave_value, '{}'.format(ave_value), color='yellow')

    ax.set_xlabel('Time [sec]')
    ax.set_ylabel('{}'.format(args.label_value))
    ax.set_title('{}'.format(args.title))
    plt.show()


if __name__ == '__main__':
    main()
