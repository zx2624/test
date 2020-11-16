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

r = []
p = []
y = []
xyz = []
with open(logfile) as fs:
    lines = fs.readlines()
    for line in lines:
        if 'r' in line:
            r.append([float(line.split()[1])])     
        if 'p' in line:
            p.append([float(line.split()[1])])   
        if 'y' in line:
            y.append([float(line.split()[1])])                                                                            
df = pd.DataFrame(r)
plt.hist(df[0], bins=100)
plt.savefig('r.png')
df = pd.DataFrame(p)
plt.hist(df[0], bins=100)
plt.savefig('p.png')
df = pd.DataFrame(y)
plt.hist(df[0], bins=100)
plt.savefig('y.png')
