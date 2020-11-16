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
            rpy.append([float(line.split()[1]) - 0 * 54, float(line.split()
                                                                  [2]) - 0 * 54, float(line.split()[3]) - 0 * 54])
        if 'xyz' in line:
            xyz.append([float(line.split()[1]) - 1, float(line.split()
                                                                  [2]) - 1, float(line.split()[3]) - 1])                                                          
dfrpy = pd.DataFrame(rpy)
dfxyz = pd.DataFrame(xyz)
plt.clf()
for i in range(3):
    plt.plot(range(len(rpy)), dfrpy[i], color=colour_list[i], label=labels[i], linewidth=1.8)
    plt.ylim(-3, 3)
plt.legend(loc=0, ncol=3, borderaxespad=0.)    
plt.savefig(logfile + '_rpy.png')

plt.clf()
for i in range(3):
    plt.plot(range(len(rpy)), dfxyz[i], color=colour_list[i], label=labels[i+3], linewidth=1.8)
    plt.ylim(-3, 3)
plt.legend(loc=0, ncol=3, borderaxespad=0.)    
plt.savefig(logfile + '_xyz.png')