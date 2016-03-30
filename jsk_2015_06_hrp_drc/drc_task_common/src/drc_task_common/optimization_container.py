#!/usr/bin/env python
import csv
import numpy as np
from scipy import signal, interpolate
import matplotlib as mpl
import matplotlib.pyplot as plt

from rospkg import RosPack
import os
def resolve_ros_path(path):
    if path.startswith("package://"):
        package_name = path.split("/")[2]
        rest_path = path.split("/")[3:]
        rp = RosPack()
        pkg_path = rp.get_path(package_name)
        return os.path.join(*([pkg_path] + rest_path))
    else:
        return path

class QualityTable():
    def __init__(self, name, file_name, q_scale):
        self.name = name
        self.file_name = resolve_ros_path(file_name)
        self.q_scale = q_scale
        # read value
        # print "Reading", file_name
        with open(self.file_name, "r") as f:
            reader = csv.reader(f)
            self.labels = reader.next()
            # self.data["q"] => [0, 0.1, ..., 1.0]
            self.data = dict()
            for label in self.labels:
                self.data[label] = []
            for row in reader:
                for i in range(len(self.labels)):
                    label = self.labels[i]
                    self.data[label].append(float(row[i]))
        # initialize linear interpolator
        times = np.array(self.data['time'])
        qs = np.array(self.data['q'])
        keys = sorted(range(0, len(times)), key=lambda x: times[x])
        self.times_sorted = np.array([times[x] for x in keys])
        self.qs_sorted = np.array([qs[x] for x in keys])
        self.time2q_interpolation = interpolate.interp1d(
            self.times_sorted, self.qs_sorted, kind="linear")
        self.q2time_interpolation = interpolate.interp1d(
            self.qs_sorted, self.times_sorted, kind="linear")
    def plot(self, c):
        range = np.linspace(self.min_time(), self.max_time(), 100)
        if self.q_scale != 1.0:
            plt.plot(range, self.q_scale * self.time2q_interpolation(range),
                     label='${1:.2f}{0}$'.format(self.name, self.q_scale),
                     lw=2, c=c)
        else:
            plt.plot(range, self.q_scale * self.time2q_interpolation(range),
                     label='${0}$'.format(self.name),
                     lw=2, c=c)
    def min_time(self):
        return self.times_sorted[0]
    def max_time(self):
        return self.times_sorted[-1]
    def min_q(self):
        return self.q_scale * self.qs_sorted[0]
    def max_q(self):
        return self.q_scale * self.qs_sorted[-1]
    def time2q(self, time):
        return self.q_scale * self.time2q_interpolation(time)
    def q2time(self, q):
        return 1.0 / self.q_scale * self.q2time_interpolation(q)
    def delta(self, time, delta_time):
        return self.time2q(time + delta_time) - self.time2q(time)
    def other_params(self):
        return [label for label in self.labels if label != "time" and label != "q"]
    def lookup_value(self, from_label, from_value, target_label):
        # run linear interpolation
        from_data = np.array(self.data[from_label])
        to_data = np.array(self.data[target_label])
        keys = sorted(range(0, len(from_data)), key=lambda x: from_data[x])
        sorted_from_data = np.array([from_data[x] for x in keys])
        sorted_to_data = np.array([to_data[x] for x in keys])
        inter = interpolate.interp1d(
            sorted_from_data, sorted_to_data, kind="linear")
        return inter(from_value)
    def lookup_value2(self, from_label1, from_label2, from_value1, from_value2, target_label):
        # run linear interpolation
        for a, b, c in zip(self.data[from_label1], self.data[from_label2], self.data[target_label]):
            if a == from_value1 and b == from_value2:
                return c

class OptimizationContainer():
    def __init__(self, no_gui, save_img):
        self.iteration_times = 0
        self.tables = []
        self.no_gui = no_gui
        self.save_img = save_img
        self.initial_times = []
        self.current_times = []
        self.tracking_times = []
        self.dt = 0.3
    def register_quality_table(self, table, initial_time):
        self.tables.append(table)
        self.initial_times.append(initial_time)
        self.current_times.append(initial_time)
        self.tracking_times.append([initial_time])
    def possible_min_time(self):
        return sum([t.min_time() for t in self.tables])
    def current_time(self):
        return sum(self.current_times)
    def current_q(self):
        return sum([t.time2q(s) for t, s in zip(self.tables, self.current_times)])
    def initial_time(self):
        return sum(self.initial_times)
    def initial_q(self):
        return sum([t.time2q(s) for t, s in zip(self.tables, self.initial_times)])
    def max_q(self):
        return sum([t.max_q() for t in self.tables])
    def is_time_bigger(self, time):
        return self.current_time() < time
    def setDirectionPositve(self):
        self.direction = 1
    def setDirectionNegative(self):
        self.direction = -1
    def delta_time(self):
        # return 0.1 * self.direction
        return self.dt * self.direction
    def is_converged(self, time_d):
        if self.direction == 1:
            return time_d < self.current_time()
        else:
            return time_d > self.current_time()
    def is_time_valid(self, tp, time):
        for t in self.tables:
            if t.name == tp:
                return time >= t.min_time() and time < t.max_time()
    def proc(self):
        self.iteration_times = self.iteration_times + 1
        # print "k=", self.iteration_times
        delta_time = self.delta_time()
        # check is valid time
        ds = []
        is_valid = False
        for t, current_time in zip(self.tables, self.current_times):
            if self.is_time_valid(t.name, current_time + delta_time):
                d = t.delta(current_time, delta_time)
                ds.append(abs(d))
                is_valid = True
            else:
                if self.direction < 0:
                    ds.append(1000.0) # large enough value
                else:
                    ds.append(-100.0)
        if not is_valid:
            # print "already converged"
            return False
        keys = sorted(range(0, len(ds)), key=lambda x: ds[x])
        if self.direction < 0:
            min_d = ds[keys[0]]
            min_table = self.tables[keys[0]]
            self.current_times[keys[0]] = self.current_times[keys[0]] + delta_time
            self.tracking_times[keys[0]].append(self.current_times[keys[0]])
        else:
            min_d = ds[keys[-1]]
            min_table = self.tables[keys[-1]]
            self.current_times[keys[-1]] = self.current_times[keys[-1]] + delta_time
            self.tracking_times[keys[-1]].append(self.current_times[keys[-1]])
        # print "choose", min_table.name
        return True
    def printOverview2Column(self):
        print "time,q"
    def printOverview2(self, deadline):
        print "{0},{1}".format(self.current_time(), self.current_q())
    def printOverview(self, deadline):
        print "initial condition:"
        print "  all time:", self.initial_time()
        print "  all q:", self.initial_q()
        for t, s in zip(self.tables, self.initial_times):
            print "  " + t.name
            print "    time:", s
            print "    q:", t.time2q(s)
            for p in t.other_params():
                print "    {0}: {1}".format(p, t.lookup_value("time",s, p))
        print "deadline time:", deadline
        print "minimum time:", sum([t.min_time() for t in self.tables])
        print "final result:"
        print "  all time:", self.current_time()
        print "  all q:", self.current_q()
        print "  rel q", self.current_q() / self.max_q()
        for t, s in zip(self.tables, self.current_times):
            print "  " + t.name
            print "    time:", s
            print "    q:", t.time2q(s)
            for p in t.other_params():
                print "    {0}: {1}".format(p, t.lookup_value("time",s, p))
    def draw(self, ax):
        if self.no_gui and not self.save_img:
            return
        ax.cla()
        ax.set_xlabel('$t$', fontsize=22)
        ax.set_ylabel('$q$', fontsize=22)
        max_time = max([t.max_time() for t in self.tables])
        max_q = max([t.max_q() for t in self.tables])
        for t, current_time, trackings, c in zip(self.tables, self.current_times,
                                                  self.tracking_times,
                                                  ["#1f77b4",
                                                   "#ff7f0e",
                                                   "#2ca02c",
                                                   "#d62728",
                                                   "#9467bd",
                                                   "#8c564b",
                                                   "#e377c2",
                                                   "#7f7f7f",
                                                   "#bcbd22",
                                                   "#17becf"]):
            current_q = t.time2q(current_time)
            t.plot(c)
            plt.plot(trackings,
                     t.time2q(trackings),
                     'o', c=c)
        ax.set_xlim(0, max_time)
        ax.set_ylim(0, max_q)
        ax.grid()
        plt.legend(loc=4)
        # plt.text(max_time * 5.0 / 8, max_q * 0.1,
        #          "$k={2}$\n$\q = {0:.1f}$\n$\time = {1:.1f}$".format(self.current_q(), self.current_time(),
        #                                                                     self.iteration_times),
        #          ha='left', va='center')
        plt.draw()

        
