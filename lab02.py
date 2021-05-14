import numpy as np
import roboticstoolbox as rtb
from spatialmath import *

robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(alpha=np.pi / 2),
        rtb.RevoluteDH(a=0.4318),
        rtb.RevoluteDH(d=0.15005, a=0.0203, alpha=-np.pi / 2),
        rtb.RevoluteDH(d=0.4318, alpha=np.pi / 2),
        rtb.RevoluteDH(alpha=-np.pi / 2),
        rtb.RevoluteDH()
    ], name="My_Robot")
robot = rtb.models.DH.Puma560()
print(robot)
# Sprawdzenie nazwy i producenta robota
print("Name: ", robot.name)
print("Manufacturer: ", robot.manufacturer)
# Sprawdzenie konfiguracji (rodzajów) przegubów (R - obrotowy, P - przesuwny)
print("Joint configuration: ", robot.structure)
# Sprawdzenie, które przeguby są obrotowe
print("Revolute joints: ", robot.revolutejoints)
# Sprawdzenie, które przeguby są przesuwne
print("Prismatic joints: ", robot.prismaticjoints)
# Sprawdzenie liczby węzłów
print("Number of joints: ", robot.n)
# Sprawdzenie czy manipulator jest opisany zmodyfikowaną notacją DH (1) lub standardową notacją DH (0)
print("MDH: ", robot.mdh)
# Sprawdzenie czy robot posiada nadgarstek sferyczny
print("Spherical wrist: ", robot.isspherical())
# Sprawdzenie maksymalnego zasięgu robota
print("Reach: ", robot.reach)
# Dodawanie własnej konfiguracji
robot.addconfiguration("mycfg", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
print(robot.mycfg)
# Konfiguracja kinematyczna: 0 -> przegub obrotowy, 1 -> przesuwny
print(robot.links[0].sigma)
# Konwencja: 0 -> standardowa DH, 1 -> zmodyfikowana DH
print(robot.links[0].mdh)
# Sprawdzenie masy członu
print(robot.links[0].m)
# Graficzne przedstawienie robota
# robot.plot(robot.qn, block = True)
# robot.teach(robot.qn) # argument jest opcjonalny
print("########")
print("########")
print("########")
T = robot.fkine(robot.q)
print(robot.q)
print(T)
q_tmp = [0.5, 0., .9, 1.2, .2, .7]
T_tmp = robot.fkine(q_tmp)
print(T_tmp)
robot.tool = SE3(0, 0, 0.05)
print(robot.fkine(robot.q))
# Zadanie
# uklad base, aby robot był zawieszony na suficie
# wysokosc sufitu 3[m]

#robot.base = SE3(0, 0, 3) #* SO3(np.array(np.array([[1, 0, 0],
                          #                       [0, 1, 0],
                          #                       [0, 0, -1]])))

#robot.plot(robot.q, block = True)
# print(robot.qn)
T = robot.fkine(robot.qn)

ik_solution = robot.ikine_a(T=T, config="ld") #konfiguracja right down
#print(ik_solution.q)
#robot.plot(ik_solution.q, block = True)
#Jakobian
J = robot.jacob0(robot.q)

np.set_printoptions(precision=3, suppress=True)
print(J)
q_tmp = [0.5, 0., 0.9, 1.2, 0.2, 0.7]
J_tmp = robot.jacob0(q_tmp)
print(J_tmp)
Je_tmp = robot.jacobe(q_tmp)
print(Je_tmp)