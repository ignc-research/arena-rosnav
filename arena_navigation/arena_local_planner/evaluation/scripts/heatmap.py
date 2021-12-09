# import numpy as np
# from matplotlib import pyplot as plt

# plt.rcParams["figure.figsize"] = [7.50, 3.50]
# plt.rcParams["figure.autolayout"] = True
# x = np.linspace(-1, 1, 1000)
# y = np.exp(x)
# c = np.tan(x)
# plt.scatter(x, y, c=c, marker='.')
# plt.show()


# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib import cm
# import matplotlib.collections as mcoll
# import matplotlib.path as mpath

# x = np.arange(-8, 4, 0.1)
# y = 1 + 0.5 * x**2

# MAP = 'jet'
# NPOINTS = len(x)

# fig = plt.figure()
# ax1 = fig.add_subplot(111) 
# cm = plt.get_cmap(MAP)
# for i in range(10):
#     ax1.set_color_cycle([cm(1.0*i/(NPOINTS-1)) for i in range(NPOINTS-1)])
#     for i in range(NPOINTS-1):
#         plt.plot(x[i:i+5],y[i:i+5],"--or")
# cl = np.tan(x)
# plt.plot(x,y)


# plt.show() # Show the figure


# import numpy as np
# import matplotlib.pylab as plt

# x = np.linspace(0, 2*np.pi, 100)
# y = np.cos(x) 

# # pl.figure()
# # pl.plot(x,y)

# n = 50
# colors = plt.cm.jet(np.linspace(0,1,n))
# vels = np.linspace(0,0.5,n)

# mid = min(range(len(vels)), key=lambda i: abs(vels[i]-0.3))


# print(vels)

# print(mid, vels[mid])

# idx = 0
# joined = 0
# xi = []
# yi = []
# skip = 2
# while idx < len(x):

#     if joined == skip or idx == len(x) - 1:
#         plt.plot(xi, yi, "-") 
#         # plt.scatter(xi, yi, marker='+', color=colors[9])
#         # plt.scatter(xi, yi, marker='+')
#         xi = []
#         yi = []
#         joined = 0
#         if idx != len(x) - 1:
#             idx -=1
#     xi.append(x[idx])
#     yi.append(y[idx])
#     # yi.append(1)

#     joined += 1
#     idx += 1

# plt.show()


import matplotlib.pyplot as plt

figure, axes = plt.subplots()
draw_circle_1 = plt.Circle((0,0), 0.1,fill = False, color = "green")
draw_circle_2 = plt.Circle((0,0), 0.3,fill = False, color = "blue")
draw_circle_3 = plt.Circle((0,0), 0.5,fill = False, color = "red")

axes.set_aspect(1)
axes.add_artist(draw_circle_1)
axes.add_artist(draw_circle_2)
axes.add_artist(draw_circle_3)
axes.set_ylim([0,0.6])
axes.set_xlim([-0.6,0.6])
plt.title('action map')
plt.show()