import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 读取CSV文件
df = pd.read_csv('data.csv',header=None)
# fig, axs = plt.subplots(3, 1)
# 提取数据
x1   = df.iloc[:, 0]
y1   = df.iloc[:, 1]
yaw1 = df.iloc[:, 2]
v    = df.iloc[:, 3]
delta_r = df.iloc[:, 4]

x2   = df.iloc[:, 5]
y2   = df.iloc[:, 6]
yaw2 = df.iloc[:, 7]
v    = df.iloc[:, 8]
delta = df.iloc[:, 9]
Delta_steer = df.iloc[:, 10]

# 绘制散点图
plt.figure(1)
plt.scatter(x1, y1, c=yaw1, cmap='hsv', label='reference path')  # 根据yaw的值设置颜色
plt.scatter(x2, y2, c=yaw2, cmap='hsv', marker='s', label='real path')
plt.colorbar(label='Yaw (deg)')  # 添加颜色条
plt.quiver(x1, y1, np.cos(yaw1), np.sin(yaw1),
           angles='xy', scale_units='xy', scale=1, color='k', label='Velocity Vector')  # 绘制速度向量
plt.quiver(x2, y2, np.cos(yaw2), np.sin(yaw2),
           angles='xy', scale_units='xy', scale=1, color='k')
# 设置标题和坐标轴标签
plt.title('Vehicle Path')
plt.xlabel('X')
plt.ylabel('Y')

# 添加图例
plt.legend()

line_x = np.arange(1, len(yaw1)+1)



plt.figure(2)
plt.plot(line_x,yaw1,label="reference")
plt.plot(line_x,yaw2,label="real")
plt.xlabel('time')
plt.ylabel('yaw')
plt.title('yaw')
plt.legend()

# 显示图形
plt.show()

plt.figure(3)
plt.plot(line_x,delta_r,label="reference")
plt.plot(line_x,delta,label="real")
plt.xlabel('time')
plt.ylabel('delta')

plt.title('delta')
plt.legend()

# 显示图形
plt.show()