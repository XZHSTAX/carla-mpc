import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
import numpy as np

def butter_lowpass(cutoff_freq, fs, order=5):
    """_summary_

    Parameters
    ----------
    cutoff_freq : _type_
        截止频率
    fs : _type_
        采样频率
    order : int, optional
        阶数, by default 5

    Returns
    -------
    _type_
        _description_
    """
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist_freq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def apply_lowpass_filter(data, cutoff_freq, fs, order=5):
    """_summary_

    Parameters
    ----------
    data : _type_
        _description_
    cutoff_freq : _type_
        截止频率
    fs : _type_
        采样频率
    order : int, optional
        阶数, by default 5

    Returns
    -------
    _type_
        处理好的数据
    """
    b, a = butter_lowpass(cutoff_freq, fs, order=order)
    filtered_data = lfilter(b, a, data)
    return filtered_data

# 使用read_csv函数读取没有表头的csv文件
data = pd.read_csv('output_new4.txt', header=None, names=['time', 'angle','m_torque' ,'ex_torque'])

time = data['time']
angle = data['angle']
m_torque = data['m_torque']
ex_torque = data['ex_torque']

Moment_inertia = 4.0
max_torque_Nm = 2.2
damping = Moment_inertia * 0.049

angle = angle*450*3.1415926/180   # 从[-1,1]映射回弧度
# 计算角速度
angle_dot = angle.diff().fillna(0)
angle_dot = angle_dot/time

# 计算角加速度
angle_ddot = angle_dot.diff().fillna(0)
angle_ddot = angle_ddot/time

ex_torque_compute = Moment_inertia * angle_ddot - m_torque*max_torque_Nm - damping * angle_dot

# 滑动平均窗口
window_size = 50  # 可根据需要调整窗口大小
angle_dot_smooth = angle_dot.rolling(window_size).mean()
angle_ddot_smooth = angle_ddot.rolling(window_size).mean()
# 低通滤波器
fs = 1/0.002
cutoff_freq = 15

angle_dot_lfilter = apply_lowpass_filter(angle_dot, cutoff_freq, fs)   # 对angle_dot使用低通滤波器
angle_ddot_lfilter = apply_lowpass_filter(angle_ddot, cutoff_freq, fs) # 对angle_ddot使用低通滤波器

# 使用滤波后的速度来计算加速度
angle_ddot_lfilter2 = np.pad(np.diff(angle_dot_lfilter), (1, 0), 'constant')/time 
ex_torque_compute_lfilter = Moment_inertia * angle_ddot_lfilter2 - m_torque*max_torque_Nm - damping * angle_dot_lfilter

plt.subplot(2, 3, 1)
plt.plot(angle.index, angle, label='angle')
plt.title(r'$\theta$')

plt.subplot(2, 3, 2)
plt.plot(angle_dot.index, angle_dot_lfilter, label='angle_dot')
plt.title(r'$\omega$')

plt.subplot(2, 3, 3)
plt.plot(angle_ddot.index, angle_ddot_lfilter2, label='angel_ddot')
plt.title(r'$\alpha$')

plt.subplot(2, 3, 4)
plt.plot(m_torque.index, m_torque, label='m_torque')
plt.title(r'machine torque')

plt.subplot(2, 3, 5)
plt.plot(ex_torque.index, ex_torque_compute_lfilter, label='ex_torque')
plt.title(r'external torque')
plt.show()