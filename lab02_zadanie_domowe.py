# wczytanie potrzebnych bibliotek:
import roboticstoolbox as rtb
import numpy as np
import math
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *
import time

def zadanie_1():
    l1 = symbol("l1")
    l2 = symbol("l2")
    robot = rtb.DHRobot(
        [
            rtb.RevoluteDH(d=l1, alpha=pi() / 2),
            rtb.RevoluteDH(alpha=pi()/2, offset=pi()/2 ),
            rtb.PrismaticDH(offset=l2)
        ], name="Mr_Robot")
    return robot

def zadanie_1_variable( l1, l2):
    robot = rtb.DHRobot(
        [
            rtb.RevoluteDH(d=l1, alpha=np.pi / 2),
            rtb.RevoluteDH(alpha=np.pi / 2, offset=np.pi / 2 ),
            rtb.PrismaticDH(offset=l2)
        ], name="Mr_Robot")
    return robot


def zadanie_2(robot):
    theta1 = symbol('1')
    theta2 = symbol('2')
    d3 = symbol('d3')
    print("Zadanie 2:")
    print(robot.fkine([theta1, theta2, d3]))

def zadanie_3(robot):
    theta1 = symbol('1')
    theta2 = symbol('2')
    d3 = symbol('d3')
    print("Zadanie 3:")
    for row in simplify(robot.jacob0(([theta1, theta2, d3]))):
        print(row)

def zadanie_4():
    robot = zadanie_1_variable(1, 0.4)
    print("Zadanie 4:")
    T = robot.fkine([0.1, 1, 0.4])
    ik_solution = robot.ikine_LM(T=T)
    print(ik_solution.q)
    print("Jest rowny q")
def zadanie_5():
    robot = rtb.models.DH.Puma560()
    T = robot.fkine(robot.qn)

    start = time.perf_counter_ns()
    ik_solution_a = robot.ikine_a(T=T)
    time_of_ikine_a = time.perf_counter_ns() - start

    #start
    ik_solution_lm = robot.ikine_LM(T=T)
    time_of_lm = time.perf_counter_ns() - time_of_ikine_a

    #start
    ik_solution_lms = robot.ikine_LMS(T=T)
    time_of_lms = time.perf_counter_ns() - time_of_lm

    #start
    ik_solution_min = robot.ikine_min(T=T, qlim=False)
    time_of_min = time.perf_counter_ns() - time_of_lms

    # start
    ik_solution_min_qlin = robot.ikine_min(T=T, qlim=True)
    time_of_min_qlin = time.perf_counter_ns() - time_of_min

    T_ikine_a = robot.fkine(ik_solution_a.q)
    print("Zadanie 5:")
    print("Blad metody ikine_a", np.linalg.norm(T - T_ikine_a))
    print("Czas metody ikine_a", time_of_ikine_a)
    print("Bblad metody ikine_LM", ik_solution_lm.residual)
    print("Czas metody ikine_LM", time_of_lm)
    print("Blad metody ikine_LMS", ik_solution_lms.residual)
    print("Czas metody ikine_LMS", time_of_lms)
    print("Blad metody ikine_min(qlim=False)", ik_solution_min.residual)
    print("Czas metody ikine_min(qlim=False)", time_of_min)
    print("Blad metody ikine_min(qlim=True)", ik_solution_min_qlin.residual)
    print("Czas metody ikine_min(qlim=True)", time_of_min_qlin)

if __name__ == '__main__':
    robot = zadanie_1()
    #print(robot.links[0])
    #print(robot.links[1])
    #print(robot.links[2])
    #zadanie_2(robot)
    zadanie_3(robot)
    #zadanie_4()
    zadanie_5()
