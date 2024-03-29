import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import cv2

imgPath = "/home/wanyel/QT_work/00_Build/build-Laser-Scanning-Qt_5_14_2_gcc_64-Debug/USER_DATA/2023_08_12_16_02_41_181.tiff"
gray_image = cv2.imread(imgPath, cv2.IMREAD_GRAYSCALE)
# 创建一个灰度图数据
# gray_image = np.array([[0, 64, 128, 192, 255],
#                       [64, 128, 192, 255, 64],
#                       [128, 192, 255, 64, 128],
#                       [192, 255, 64, 128, 192],
#                       [255, 64, 128, 192, 255]])

# 创建一个网格
x, y = np.meshgrid(np.arange(0, gray_image.shape[1]), np.arange(0, gray_image.shape[0]))

# 创建一个3D图形对象
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制灰度图的三维可视化
ax.plot_surface(x, y, gray_image, cmap='hot')
# ax.plot_wireframe(x, y, gray_image, cmap='jet')

# 设置坐标轴标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('GrayValue')

# 显示图形
plt.show()
