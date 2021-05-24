import numpy as np
from spatialmath import *
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import *
import time
from roboticstoolbox.backends.Swift import Swift

# wykres a)
#traj = tpoly(0, 1, 50) # trajektoria w zakresie od 0 do 1 z 50 próbek
#traj.plot(traj)

# wykres b)
#traj = tpoly(0, 1, 50, 0.5) # trajektoria z niezerową prędkością początkową
#traj.plot(traj)

#wykres c)
#traj = lspb(0, 1, 50)
#traj.plot(traj)
#print(max(traj.qd))
#traj = lspb(0, 1, 50, 0.03)
#traj.plot(traj)
# trajektoria wielomianowa
traj = jtraj([0, 2], [1, -1], 50)
#rtb.qplot(traj.q, block=True)
# lub
traj = mtraj(tpoly, [0, 2], [1, -1], 50)
#traj.plot(traj)
traj = mtraj(lspb, [0, 2], [1, -1], 50)
#traj.plot(traj)
robot = rtb.models.DH.Panda()
#traj = jtraj(robot.qz, robot.qr, 50)
# wykres pozycji:
#rtb.qplot(traj.q,  block=True)
# wykres prędkości:
#rtb.qplot(traj.qd,  block=True)
# wykres przyspieszeń:
#rtb.qplot(traj.qdd, block=True)
# Przykład dla robota o 2 stopniach swobody, trajektoria składa się z 4 punktów
#via_pt = np.array([[0,0], [1,0.5], [0.2,2], [0.5,1]])
#traj = mstraj(via_pt, dt=0.02, tacc=0.2, qdmax=2.0)
#rtb.qplot(traj.q, block=True)
T1 = SE3(0.4, 0.2, 0) * SE3.RPY([0, 0, 3])
T2 = SE3(-0.4, -0.2, 0.3) * SE3.RPY([-np.pi / 4, np.pi / 4, -np.pi / 2])
cart_traj = ctraj(T1, T2, 50)
#print(cart_traj)
# PyPlot
robot = rtb.models.DH.Panda()
T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
solution = robot.ikine_LM(T)
traj = jtraj(robot.qz, solution.q, 50)
#robot.plot(traj.q, backend = 'pyplot', movie='panda_pyplot.gif')
# Swift
robot = rtb.models.Panda()
T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
solution = robot.ikine_LM(T)
traj = jtraj(robot.qz, solution.q, 50)
robot.plot(traj.q, backend = 'swift')


# Make and instance of the Swift simulator and open it
env = Swift()
env.launch()

# Make a robot model and set its joint angles to the ready joint configuration
robot = rtb.models.Panda()
robot.q = robot.qr

# Set a desired and effector pose an an offset from the current end-effector pose
Tep = robot.fkine(robot.q) * SE3.Tx(0.2) * SE3.Ty(0.2) * SE3.Tz(0.45)

# Add the robot to the simulator
env.add(robot)
time.sleep(3)

# Simulate the robot while it has not arrived at the goal
arrived = False
while not arrived:
    # Work out the required end-effector velocity to go towards the goal
    v, arrived = rtb.p_servo(robot.fkine(robot.q), Tep, 1)

    # Set the robot's joint velocities (calculate pseudoinverse(J) * v)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v

    # Step the simulator by 15 milliseconds
    env.step(0.015)