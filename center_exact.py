import cv2
import numpy as np
import matplotlib.pyplot as plt

# 读取图像(1024, 1536, 3)
img_path = "srcImg/bmp/test4.bmp"
img = cv2.imread(img_path)
print(img.shape)

# img=img[100:400,450:1000]
# cv2.imshow('img',img)
# cv2.waitKey(0)

emptyImage3 = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret1,th1 = cv2.threshold(emptyImage3,100,255,cv2.THRESH_BINARY)
# cv2.imshow('img',th1)
# cv2.waitKey(0)

# 灰度重心法提取光条纹公式函数
def cvpointgray(img):
    x = []
    for i in range(img.shape[1]):
        x0 = y0 = 0
        for j in range(1,img.shape[0]):
            x0 += img[j,i]
            y0 += img[j,i] * j
            
        y = round(y0/x0)
        x.append(int(y))
    return x
    
w_list = range(img.shape[1])
h_list = cvpointgray(th1)
w_h_coors = zip(w_list, h_list)
# print(list(w_h_coors))	# 使用list会改变原有列表
# w_point = len(h_list)
# print(w_point)			# W方向的点数（1536个）
# print(h_list)	# 激光线中心位置所有坐标（H方向，<0, 1024>）

# 绘图
# plt.plot(w_list, h_list)
# plt.show()

for p in w_h_coors:
	print(p)
	cv2.circle(img, p, 0, (0, 0, 255), -1)

# 保存图像
save_name = img_path.split('/')[2]
cv2.imwrite(f'resultImg/line_{save_name}', img)
cv2.imshow('img',img)
cv2.waitKey(0)
