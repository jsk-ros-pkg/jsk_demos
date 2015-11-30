#!/usr/bin/env python
# coding: UTF-8

import sys
import csv
import time
import numpy as np
from scipy import signal, interpolate
import matplotlib as mpl
import matplotlib.pyplot as plt
from optimization_container import QualityTable, OptimizationContainer
plt.ion()                       # enable interactive mode
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--p0', default=1.0, type=float)
parser.add_argument('--m0', default=1.0, type=float)
parser.add_argument('--p1', default=1.0, type=float)
parser.add_argument('--m1', default=1.0, type=float)
parser.add_argument('--e1', default=1.0, type=float)
parser.add_argument('sigma', type=float)
args = parser.parse_args()
deadline_sigma = args.sigma

# global variables
# door
perception_csv = "epsilon_plane_sigma.csv"
planning_csv = "ik_sigma_epsilon.csv"
execution_csv = "zmp-door_epsilon_sigma.csv"
all_planning_csv = "ik_sigma_epsilon_all.csv"
initial_dx = 10.0               # 1cm downsample is default
initial_collision = 5
initial_traj = 12
initial_execution_sigma = 20.0198

ax = plt.figure().add_subplot(111)
ax.set_xlabel('$\sigma$')
ax.set_ylabel('$\epsilon$')

# check direction
container = OptimizationContainer()


p0_table = QualityTable("p0", "epsilon_plane_sigma.csv", args.p0)
m0_table = QualityTable("m0", "ik_sigma_epsilon.csv", args.m0)
m0_all_table = QualityTable("m0", "ik_sigma_epsilon_all.csv", args.m0)
p1_table = QualityTable("p1", "epsilon_plane_sigma.csv", args.p1)
m1_table = QualityTable("m1", "ik_sigma_epsilon.csv", args.m1)
e1_table = QualityTable("e1", "zmp-door_epsilon_sigma.csv", args.e1)


if args.p0 != 0.0:    
    container.register_quality_table(p0_table, p0_table.lookup_value('dx', initial_dx, 'sigma'))
if args.m0 != 0.0:
    container.register_quality_table(m0_table, m0_all_table.lookup_value2('collision', 'trajectory',
                                                                          initial_collision, initial_traj,
                                                                          'sigma'))
if args.p1 != 0.0:
    container.register_quality_table(p1_table, p1_table.lookup_value('dx', initial_dx, 'sigma'))
if args.m1 != 0.0:
    container.register_quality_table(m1_table, m0_all_table.lookup_value2('collision', 'trajectory',
                                                                          initial_collision, initial_traj,
                                                                          'sigma'))
if args.e1 != 0.0:
    container.register_quality_table(e1_table, initial_execution_sigma)

print "Initial sigma:", container.current_sigma()
if container.current_sigma() < deadline_sigma:
    print "Increase sigma"
    container.setDirectionPositve()
else:
    print "Decrease sigma"
    container.setDirectionNegative()
raw_input()    
while not container.is_converged(deadline_sigma):
    if not container.proc():
        container.draw(ax)
        break
    container.draw(ax)
print "epsilon =", container.current_epsilon()
raw_input()
