import numpy as np

p = 0.0024              # 像元大小mm
a2 = 50                 # 近端沙姆结构夹角，？？
p1 = p * np.sin(a2)
a = 16.62                 # 夹角 25.34 16.62
h = 248                 # 高度 154 248
u = h / np.cos(a)       # 物距 
v = 27.94               # 像距？？？

hz = u * p / (v * np.sin(a) + p * np.cos(a))

print(hz)