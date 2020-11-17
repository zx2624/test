import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

logfile = sys.argv[1]
colour_list = [
    '#000000', '#8A2BE2', '#DC143C', '#556B2F', '#483D8B', '#1E90FF',
    '#FFD700', '#E6E6FA', '#B0C4DE', '#0000CD'
]
labels = 'rpyxyz'

rpy = []
xyz = []
with open(logfile) as fs:
    lines = fs.readlines()
    for line in lines:
        if 'rpy' in line:
            rpy.append([float(line.split()[1]) * 54, float(line.split()
                                                           [2]) * 54, float(line.split()[3]) * 54])
        if 'xyz' in line:
            xyz.append([float(line.split()[1]), float(line.split()
                                                      [2]), float(line.split()[3])])
dfrpy = pd.DataFrame(rpy)
dfxyz = pd.DataFrame(xyz)
begin = 0 
end = len(rpy)
if len(sys.argv) > 2:
    begin = int(sys.argv[2])
    end = int(sys.argv[3])
plt.clf()
for i in range(3):
    plt.plot(range(end-begin),
             dfrpy.iloc[begin:end, i], color=colour_list[i], label=labels[i], linewidth=1.8)
    plt.ylim(-3, 3)
plt.legend(loc=0, ncol=3, borderaxespad=0.)
plt.savefig(logfile + '_rpy.png')

plt.clf()
for i in range(3):
    plt.plot(range(end-begin),
             dfxyz.iloc[begin:end, i], color=colour_list[i], label=labels[i+3], linewidth=1.8)
    plt.ylim(-3, 3)
plt.legend(loc=0, ncol=3, borderaxespad=0.)
plt.savefig(logfile + '_xyz.png')
