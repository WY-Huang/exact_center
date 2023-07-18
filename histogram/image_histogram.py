import os
import sys

import cv2
import numpy as np
import matplotlib.pyplot as plt

shizi_img = cv2.imread("/home/wanyel/vs_code/exact_center/histogram/data/2023_07_06_14_58_14_043.bmp")
print(shizi_img.shape)
plt.imshow(shizi_img)

# [..., ::-1]
colors = ["blue", "green", "red"]

for i in range(3):
    # np.histogram() 生成直方图， raval()将多维数组降成一维， bins 横轴区间个数， range 统计范围
    # hist 返回每个区间的个数， x 横轴的值
    hist, x = np.histogram(shizi_img[:, :, i].ravel(), bins=256, range=(0, 256))
    plt.plot(0.5 * (x[:-1] + x[1:]), hist, label=colors[i], color=colors[i])
    
plt.show()
# cal_x = 0.5 * (x[:-1] + x[1:])    # 计算区间的中值
# print(cal_x, cal_x.dtype)])      # BGR--> RGB 这是什么魔法？

shizi_gray = cv2.imread("data/shizi.bmp", cv2.IMREAD_GRAYSCALE)

plt.imshow(shizi_gray, cmap="gray")

hist_gray, x_gray = np.histogram(shizi_gray.ravel(), bins=256, range=(0, 256))
plt.plot(0.5 * (x_gray[:-1] + x_gray[1:]), hist_gray)
plt.show()