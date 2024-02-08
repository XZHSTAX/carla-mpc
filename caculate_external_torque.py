import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
import numpy as np

Moment_inertia = 0.00767
max_torque_Nm = 2.1
damping = Moment_inertia * 24.30

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

def caculated_ex_torque(angle_dot,angle_ddot):
    return Moment_inertia * angle_ddot - m_torque*max_torque_Nm + damping * angle_dot

# 使用read_csv函数读取没有表头的csv文件
data = pd.read_csv('output_new4.txt', header=None, names=['time', 'angle','m_torque' ,'ex_torque'])

time = data['time']
angle = data['angle']
m_torque = data['m_torque']
ex_torque = data['ex_torque']

angle = angle*450*3.1415926/180   # 从[-1,1]映射回弧度
# 计算角速度
angle_dot = angle.diff().fillna(0)
angle_dot = angle_dot/time

# 计算角加速度
angle_ddot = angle_dot.diff().fillna(0)
angle_ddot = angle_ddot/time

# ex_torque_compute = Moment_inertia * angle_ddot - m_torque*max_torque_Nm - damping * angle_dot
ex_torque_compute = caculated_ex_torque(angle_dot,angle_ddot)
# 滑动平均窗口
window_size = 50  # 可根据需要调整窗口大小
angle_dot_smooth = angle_dot.rolling(window_size).mean()
angle_ddot_smooth = angle_ddot.rolling(window_size).mean()
angle_ddot_smooth2 = angle_dot_smooth.diff().fillna(0)
# 低通滤波器
fs = 1/0.002
cutoff_freq = 15

angle_dot_lfilter = apply_lowpass_filter(angle_dot, cutoff_freq, fs)   # 对angle_dot使用低通滤波器
angle_ddot_lfilter = apply_lowpass_filter(angle_ddot, cutoff_freq, fs) # 对angle_ddot使用低通滤波器

# 使用滤波后的速度来计算加速度
angle_ddot_lfilter2 = np.pad(np.diff(angle_dot_lfilter), (1, 0), 'constant')/time 

ex_torque_compute_lfilter  = caculated_ex_torque(angle_dot_lfilter,angle_ddot_lfilter) 
ex_torque_compute_lfilter2 = caculated_ex_torque(angle_dot_lfilter,angle_ddot_lfilter2)

ex_torque_compute_smooth   = caculated_ex_torque(angle_dot_smooth,angle_ddot_smooth)
ex_torque_compute_smooth2  = caculated_ex_torque(angle_dot_smooth,angle_ddot_smooth2)

# 第一个图用来对比原始数据，滑动平均窗口数据和低通滤波数据；通过此图，可以看出不同的滤波，对原始加速度数据的效果如何
fig1 = plt.figure()
ax1 = fig1.add_subplot(3, 3, 1)
ax1.plot(angle_dot.index, angle_dot, label='angle_dot')
ax1.set_title(r'$\omega$')

ax2 = fig1.add_subplot(3, 3, 2)
ax2.plot(angle_ddot.index, angle_ddot, label='angel_ddot')
ax2.set_title(r'$\alpha$')

ax3 = fig1.add_subplot(3, 3, 4)
ax3.plot(angle_dot.index, angle_dot_smooth, label='angle_dot_smooth')
ax3.set_title(r'$\omega$-smooth')

ax4 = fig1.add_subplot(3, 3, 5)
ax4.plot(angle_ddot.index, angle_ddot_smooth, label='angel_ddot_smooth')
ax4.set_title(r'$\alpha$-smooth')

ax8 = fig1.add_subplot(3, 3, 6)
ax8.plot(angle_ddot.index, angle_ddot_smooth2, label='angel_ddot_smooth2')
ax8.set_title(r'$\alpha$-smooth2')

ax5 = fig1.add_subplot(3, 3, 7)
ax5.plot(angle_dot.index, angle_dot_lfilter, label='angle_dot_lfilter')
ax5.set_title(r'$\omega$-filter')

ax6 = fig1.add_subplot(3, 3, 8)
ax6.plot(angle_ddot.index, angle_ddot_lfilter, label='angel_ddot_filter')
ax6.set_title(r'$\alpha$-filter')

ax7 = fig1.add_subplot(3, 3, 9)
ax7.plot(angle_ddot.index, angle_ddot_lfilter2, label='angel_ddot_filter2')
ax7.set_title(r'$\alpha$-filter2')
# -------------------------------------------------------------------------
# 第二张图用来对比所计算的外部力矩；通过此图，可以看出不同的滤波对最终力矩的计算效果如何
fig2 = plt.figure()

ax1 = fig2.add_subplot(2, 3, 1)
ax1.plot(ex_torque.index, ex_torque, label='ex_torque')
ax1.set_title(r'external torque(origin)')

ax2 = fig2.add_subplot(2, 3, 4)
ax2.plot(ex_torque_compute.index, ex_torque_compute, label='ex_torque_compute')
ax2.set_title(r'external torque(origin compute)')

ax3 = fig2.add_subplot(2, 3, 2)
ax3.plot(ex_torque_compute_smooth.index, ex_torque_compute_smooth, label='ex_torque_compute_smooth')
ax3.set_title(r'external torque(smooth)')

ax4 = fig2.add_subplot(2, 3, 5)
ax4.plot(ex_torque_compute_smooth.index, ex_torque_compute_smooth2, label='ex_torque_compute_smooth2')
ax4.set_title(r'external torque(smooth2)')

ax5 = fig2.add_subplot(2, 3, 3)
ax5.plot(ex_torque_compute_lfilter.index, ex_torque_compute_lfilter, label='ex_torque_compute_lfilter')
ax5.set_title(r'external torque(filter)')

ax6 = fig2.add_subplot(2, 3, 6)
ax6.plot(ex_torque_compute_lfilter.index, ex_torque_compute_lfilter2, label='ex_torque_compute_lfilter2')
ax6.set_title(r'external torque(filter2)')

# 第三幅图，对比各个加速度与所计算的力矩；通过此图，可以看出加速度的结果，对力矩的计算其决定性作用；不同滤波算法下，加速度和力矩的关系
fig3 = plt.figure()

ax1 = fig3.add_subplot(2, 3, 1)
ax1.plot(angle_ddot.index, angle_ddot, label='angel_ddot')
ax1.set_title(r'$\alpha$')

ax2 = fig3.add_subplot(2, 3, 4)
ax2.plot(ex_torque.index, ex_torque, label='ex_torque')
ax2.set_title(r'external torque(origin)')

ax3 = fig3.add_subplot(2, 3, 2)
ax3.plot(angle_ddot.index, angle_ddot_smooth, label='angel_ddot_smooth')
ax3.set_title(r'$\alpha$-smooth')

ax4 = fig3.add_subplot(2, 3, 5)
ax4.plot(ex_torque_compute_smooth.index, ex_torque_compute_smooth, label='ex_torque_compute_smooth')
ax4.set_title(r'external torque(smooth)')

ax5 = fig3.add_subplot(2, 3, 3)
ax5.plot(angle_ddot.index, angle_ddot_smooth2, label='angel_ddot_smooth2')
ax5.set_title(r'$\alpha$-smooth2')

ax6 = fig3.add_subplot(2, 3, 6)
ax6.plot(ex_torque_compute_smooth.index, ex_torque_compute_smooth2, label='ex_torque_compute_smooth2')
ax6.set_title(r'external torque(smooth2)')

# 第四幅图，继续对比各个加速度与所计算的力矩
fig4 = plt.figure()
ax1 = fig4.add_subplot(2, 2, 1)
ax1.plot(angle_ddot.index, angle_ddot_lfilter, label='angel_ddot_filter')
ax1.set_title(r'$\alpha$-filter')

ax2 = fig4.add_subplot(2, 2, 3)
ax2.plot(ex_torque_compute_lfilter.index, ex_torque_compute_lfilter, label='ex_torque_compute_lfilter')
ax2.set_title(r'external torque(filter)')

ax3 = fig4.add_subplot(2, 2, 2)
ax3.plot(angle_ddot.index, angle_ddot_lfilter2, label='angel_ddot_filter2')
ax3.set_title(r'$\alpha$-filter2')

ax4 = fig4.add_subplot(2, 2, 4)
ax4.plot(ex_torque_compute_lfilter.index, ex_torque_compute_lfilter2, label='ex_torque_compute_lfilter2')
ax4.set_title(r'external torque(filter2)')

# -------------------------------------------------------------------------
fig = plt.figure()
# 创建第一个子图
ax1 = fig.add_subplot(2, 3, 1)
ax1.plot(angle.index, angle, label='angle')
ax1.set_title(r'$\theta$')

# 创建第二个子图
ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(angle_dot.index, angle_dot_lfilter, label='angle_dot')
ax2.set_title(r'$\omega$')

# 创建第三个子图
ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(angle_ddot.index, angle_ddot_lfilter2, label='angel_ddot')
ax3.set_title(r'$\alpha$')

# 创建第四个子图
ax4 = fig.add_subplot(2, 3, 4)
ax4.plot(m_torque.index, m_torque, label='m_torque')
ax4.set_title(r'machine torque')

# 创建第五个子图
ax5 = fig.add_subplot(2, 3, 5)
ax5.plot(ex_torque.index, ex_torque_compute_lfilter, label='ex_torque(caculated)')
ax5.set_title(r'external torque(caculated)')

# 创建第六个子图
ax6 = fig.add_subplot(2, 3, 6)
ax6.plot(ex_torque.index, ex_torque, label='ex_torque')
ax6.set_title(r'external torque')

# 调整子图之间的间距
plt.subplots_adjust(wspace=0.4, hspace=0.4)

plt.show()