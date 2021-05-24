import matplotlib.pyplot as plt
import numpy as np
from spatialmath import *
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import *
import time

radius = .1
x_radius = .65
y_radius = .2
z_radius = .15
t = np.linspace(0, 2*np.pi, 36)
x = radius * np.cos(t) + x_radius
y = radius * np.sin(t) + y_radius

robot = rtb.models.DH.Panda()
q_start = robot.qz
T_start = robot.fkine(q_start)

T = SE3(x_radius, y_radius, z_radius) * SE3.Rx(np.pi)
solution = robot.ikine_LM(T)
via_points = np.array([q_start, solution.q])
solution_circle = []
for j in range(len(t)):
    T_tmp = SE3( x[j], y[j], z_radius) * SE3.Rx(np.pi)
    solution_circle.append(robot.ikine_LM(T_tmp).q)
solution_circle = np.array(solution_circle)
via_points = np.append(via_points, solution_circle, axis=0)
plt.scatter(x,y)
plt.show()
traj = mstraj(via_points, dt=0.1, tacc=0.05, qdmax=.5)
rtb.qplot(traj.q, block=True)
robot.plot(traj.q, backend="pyplot")
#robot.plot(traj.q, backend="pyplot", movie='panda_swift.gif')