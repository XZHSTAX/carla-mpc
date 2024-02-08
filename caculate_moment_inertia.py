import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np

def fit_func(t, k, a):
    return a*(1 - np.exp(-k * t))
# 读取txt文件
data = pd.read_csv('output_no_ex_force.txt', header=None,names=['time', 'angle','m_torque' ,'ex_torque'])

time = data['time']            # 间隔时间
angle = data['angle']
cumulative_sum = time.cumsum() # 记录的时刻

angle = angle*450*3.1415926/180
# 计算相邻两个数值之间的差值
data_dot = angle.diff().fillna(0)
data_dot = data_dot/time
# 计算data_dot相邻两个数的差值
data_ddot = data_dot.diff().fillna(0)
data_ddot = data_ddot/time

# window_size = 20  # 可根据需要调整窗口大小
# data_dot_smooth = data_dot.rolling(window_size).mean()
# data_ddot_smooth = data_ddot.rolling(window_size).mean()
# 绘制折线图
# plt.plot(data.index, data[0], label='Data')
plt.plot(cumulative_sum[:200], data_dot[:200], label='Data_dot')
# plt.plot(data_ddot.index, data_ddot[0], label='Data_ddot')

# 添加标题和轴标签
plt.title(r'$\dot{\theta} - t$')
plt.xlabel(r'$t$')
plt.ylabel(r'$\dot{\theta}$')

# # # 添加图例
# # plt.legend()
# plt.subplot(1, 3, 1)
# plt.plot(data.index, data[0])
# plt.title(r'$\theta$')

# # 第二张图
# plt.subplot(1, 3, 2)
# plt.plot(data_dot.index, data_dot[0])
# plt.title(r'$\dot{\theta}$')

# # 第三张图
# plt.subplot(1, 3, 3)
# plt.plot(data_ddot.index, data_ddot[0])
# plt.title(r'$\ddot{\theta}$')

# plt.tight_layout()

t = cumulative_sum[:100]
popt, pcov = curve_fit(fit_func, t, data_dot[:100])
print("a = ", popt[1]," k = ",popt[0])

t_fit = cumulative_sum[:200]
y_fit = fit_func(t_fit, popt[0],popt[1])
plt.plot(t_fit, y_fit, 'r-', label='fitting')



# 展示图表
plt.show()
