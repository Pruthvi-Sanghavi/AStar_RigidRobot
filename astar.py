import numpy as np
import matplotlib.pyplot as plot
from matplotlib.animation import FuncAnimation
# map = np.zeros((300, 200))
obstaclesx = []
obstaclesy = []
# Enter the starting node coordinates
xs = input("Enter the starting node x coordinate: ")
ys = input("Enter the starting node y coordinate: ")

# Enter the goal node coordinates
xg = input("Enter the goal node x coordinate: ")
yg = input("Enter the goal node y coordinate: ")
plot.axis([0, 302, 0, 202])
for x in range(302):
    for y in range(202):
        if (x >= 0 and x <=1):
            obstaclesx.append(x)
            obstaclesy.append(y)
        if (x >= 301 and x <= 302):
            obstaclesx.append(x)
            obstaclesy.append(y)
        if (y >= 0 and y <= 1):
            obstaclesx.append(x)
            obstaclesy.append(y)
        if (y >= 201 and y <= 202):
            obstaclesx.append(x)
            obstaclesy.append(y)


        # if ((x - 225) ** 2 + (y - 150) ** 2 <= 25 ** 2):
        #     # Map[200 - y][x] = 1
        #     obstaclesx.append(x)
        #     obstaclesy.append(y)
        # if (((x - 150) ** 2 / 40 ** 2 + (y - 100) ** 2 / 20 ** 2) <= 1):
        #     # Map[200 - y][x] = 1
        #     obstaclesx.append(x)
        #     obstaclesy.append(y)
        # if ((y - (0.6 * x)) >= (-125) and (y - (-0.6 * x)) <= (175) and (y - (0.6 * x)) <= (-95) and (
        #         y - (-0.6 * x)) >= (145)):
        #     # Map[200 - y][x] = 1
        #     obstaclesx.append(x)
        #     obstaclesy.append(y)
        # if ((y - (13 * x)) <= (-140) and (y - (1 * x)) >= (100) and y <= 185 and (y - (1.4 * x) >= 80)):
        #     # Map[200 - y][x] = 1
        #     obstaclesx.append(x)
        #     obstaclesy.append(y)
        # if ((y - (-1.2 * x)) >= (210) and (y - (1.2 * x)) >= (30) and (y - (-1.4 * x)) <= (290) and (
        #         y - (-2.6 * x)) >= 280 and y <= 185):
        #     # Map[200 - y][x] = 1
        #     obstaclesx.append(x)
        #     obstaclesy.append(y)
        # if ((y - (1.73) * x + 135 >= 0) and (y + (0.58) * x - 96.35 <= 0) and (y - (1.73) * x - 15.54 <= 0) and (
        #         y + (0.58) * x - 84.81 >= 0)):
        #     # Map[200 - y][x] = 1
        #     obstaclesx.append(x)
        #     obstaclesy.append(y)


plot.scatter(obstaclesx, obstaclesy)
plot.scatter(xs,ys)
plot.scatter(xg, yg)
plot.arrow(xs,ys,xg,yg)
plot.grid('off')
plot.show()