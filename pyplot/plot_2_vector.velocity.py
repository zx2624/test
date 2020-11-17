#!/usr/bin/env python2.7
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd

file_num = len(sys.argv) - 1
colours = ['#000000',
           '#8A2BE2',
           '#DC143C',
           '#556B2F',
           '#483D8B',
           '#1E90FF',
           '#FFD700',
           '#E6E6FA',
           '#B0C4DE',
           '#0000CD']
begin = 0
v_s_x_compute = []
v_s_x_opt = []
v_s_y_compute = []
v_s_y_opt = []
v_s_z_compute = []
v_s_z_opt = []
for file_i in range(1, file_num + 1):
    ls = [l.split() for l in open(sys.argv[file_i])]
    x_s = map(float, [l[begin + 0] for l in ls])
    y_s = map(float, [l[begin + 1] for l in ls])
    if file_i == 1:
        v_s_x_compute = map(float, [l[begin + 3] for l in ls])
        v_s_y_compute = map(float, [l[begin + 4] for l in ls])
        v_s_z_compute = map(float, [l[begin + 5] for l in ls])
    else:
        v_s_x_opt = map(float, [l[begin + 3] for l in ls])
        v_s_y_opt = map(float, [l[begin + 4] for l in ls])
        v_s_z_opt = map(float, [l[begin + 5] for l in ls])
    xs = []
    ys = []
    v_x_s = []
    v_y_s = []
    vdiff_x_s = []
    vdiff_y_s = []
    vdiff_z_s = []
    for i in range(0, len(x_s)):
        x = x_s[i]
        y = y_s[i]
        v_x = v_s_x_compute[i]
        v_y = v_s_y_compute[i]
        if file_i > 1:
	    vdiff_x_s.append(v_s_x_opt[i + 1] - v_s_x_compute[i])
	    vdiff_y_s.append(v_s_y_opt[i + 1] - v_s_y_compute[i])
	    vdiff_z_s.append(v_s_z_opt[i + 1] - v_s_z_compute[i])
        if abs(x) < 10 or abs(y) < 10:
            continue
        xs.append(x)
        ys.append(y)
        v_x_s.append(v_x)
        v_y_s.append(v_y)
    df_x = pd.DataFrame(vdiff_x_s)
    df_y = pd.DataFrame(vdiff_y_s)
    df_z = pd.DataFrame(vdiff_z_s)
    if file_i > 1:
        print (df_x.describe())
        print (df_y.describe())
        print (df_z.describe())
    x_s = xs[0:4000]
    y_s = ys[0:4000]
    v_s_x_opt = v_x_s[0:4000]
    v_s_y_opt = v_y_s[0:4000]
    # plt.quiver(x_s, y_s, cos_s, sin_s, units='dots', color=colours[file_i], label=str(file_i))
    plt.quiver(x_s, y_s, v_s_x_opt, v_s_y_opt, angles='xy', scale_units='xy',
               scale=1, color=colours[file_i - 1], label=str(file_i))
plt.axis('equal')
plt.legend()
plt.show()


# ls = [l.split() for l in open(sys.argv[2])]
# x_s_2 = map(float, [l[begin + 1] for l in ls])
# y_s_2 = map(float, [l[begin + 2] for l in ls])
# yaw_s_2 = map(float, [l[begin + 6] for l in ls])

# xs = []
# ys = []
# yaws = []
# for i in range(0, len(x_s_2)):
# 	x = x_s_2[i]
# 	y = y_s_2[i]
# 	yaw = yaw_s_2[i]
# #	if abs(x) < 10 or abs(y) < 10:
# #	continue
# 	xs.append(x)
# 	ys.append(y)
# 	yaws.append(yaw)

# x_s_2 = xs
# y_s_2 = ys
# yaw_s_2 = yaws

# ls = [l.split() for l in open(sys.argv[2])]
# xs_new = map(float, [l[begin + 7] for l in ls])
# ys_new = map(float, [l[begin + 11] for l in ls])

# ls = [l.split() for l in open(sys.argv[3])]
# xs = map(float, [l[begin + 1] for l in ls])
# ys = map(float, [l[begin + 2] for l in ls])

# a,b=xs[0],ys[0]
# ga,gb=gxs[0],gys[0]

# for i in range(len(xs)):
#xs[i] = xs[i]-a
#ys[i] = ys[i]-b

# for i in range(len(gxs)):
#gxs[i] =gxs[i]-ga
#gys[i] = gys[i]-gb

# for i in range(len(xs_new)):
# # xs_new[i] = xs_new[i]
# ys_new[i] = ys_new[i] + 10

# value = np.asarray(xs_good)
# plt.hist(value * 100 )
# plt.title("Gaussian Histogram")
# plt.xlabel("Value")
# plt.ylabel("Frequency")


# cos_s = np.cos(yaw_s)
# sin_s = np.sin(yaw_s)


# cos_s_2 = np.cos(yaw_s_2)
# sin_s_2 = np.sin(yaw_s_2)

# plt.quiver(x_s_2, y_s_2, cos_s_2, sin_s_2, units='width',color="g", label="2")

# plt.scatter(xs, ys, color = 'red', label="gnss")

# plt.scatter(xs_new, ys_new, color = 'blue', label="loam")
# plt.plot(xs, ys, 'g', label="carto")
# plt.axis('equal')
# plt.legend()
# plt.show()
