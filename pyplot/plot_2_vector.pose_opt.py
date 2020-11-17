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
start_num = 0
end_num = 145
for file_i in range(1, file_num + 1):
    ls = [l.split() for l in open(sys.argv[file_i])]
    x_s = list(map(float, [l[begin + 0] for l in ls]))
    y_s = list(map(float, [l[begin + 1] for l in ls]))
    yaw_s = list(map(float, [l[begin + 5] for l in ls]))
    dyaw = 0  # singleyaw - yaw_s[199]
    xs = []
    ys = []
    yaws = []
    yawdiff = []
    for i in range(0, len(x_s)):#(start_num,end_num)
        x = x_s[i]
        y = y_s[i]
        yaw = yaw_s[i]
        roadyaw = 0
        if i + 1 < len(x_s) and file_i > 1:
            dx = x_s[i+1] - x_s[i]
            dy = y_s[i+1] - y_s[i]
            roadyaw = math.atan2(dy, dx)
            diffyaw = math.asin(math.sin(yaw - roadyaw)) * 54
            if(file_i > 1):
                if math.sqrt(math.pow(dy, 2) + math.pow(dx, 2)) > 0.3:
                    yawdiff.append(diffyaw)
                    # print('yaw%f roadyaw%f dy:%f dx:%f --- diffyaw:%f' % (yaw, roadyaw, dy, dx, diffyaw))
            #     if 1:
            #         print(x_s[i], y_s[i])
            # print('yaw diff %f \n' % diffyaw * 54)
        if abs(x) < 10 or abs(y) < 10:
            continue
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
    print(len(yawdiff))
    df = pd.DataFrame(yawdiff)
    if file_i > 1:
        print (df.describe())
        # if abs(yaw + dyaw + roadyaw) * 54 > 10:
        #     print ('yaw1 %f   yaw2  %f' % ((yaw + dyaw) * 54, roadyaw * 54))
        #     print ('y1:%f y0:%f x1:%f x0:%f' %(y_s[i+1] ,y_s[i] , x_s[i+1] , x_s[i]))
    # df = pd.DataFrame(yawdiff)

    # for col in df.columns:
    #     print('-------------')
    #     print(df[col].mean())
    #     print(df[col].max())
    x_s = xs#[start_num:end_num]
    y_s = ys#[start_num:end_num]
    yaw_s = yaws#[start_num:end_num]
    cos_s = np.cos(yaw_s)
    sin_s = np.sin(yaw_s)
    # plt.quiver(x_s, y_s, cos_s, sin_s, units='dots', color=colours[file_i], label=str(file_i))
    plt.quiver(x_s, y_s, cos_s, sin_s, angles='xy', scale_units='xy',
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
