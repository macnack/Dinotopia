# wczytanie potrzebnych bibliotek:
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import numpy as np
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *
from numpy.linalg import inv
# ...

# definicje funkcji:
#def przykład_1():


def zadanie_1():
    R = SO3.Rz(-np.pi / 3) * SO3.Ry(np.pi / 6) * SO3.Rx(-np.pi / 4)
    Rot = SO3.RPY(-np.pi / 4, np.pi / 6, -np.pi / 3)
    print(np.linalg.det(R.A) == 1)  # wyznacznik macierzy
    print(np.linalg.inv(R.A))
    print(np.transpose(R.A))
    print(np.transpose(R.A) == np.linalg.inv(R.A))  # wlasnoscc macierzy
    n = np.array(R.A[:, 0])
    o = np.array(R.A[:, 1])
    a = np.array(R.A[:, 2])
    print(round(np.dot(n.T, o)) == round(np.dot(o.T, a)) == round(
        np.dot(a.T, n)) == 0)  # iloczyn skalarny ortogonalnych wektorow
    print(np.linalg.norm(n) == np.linalg.norm(o) == np.linalg.norm(a) == 1)  # norma wektora = 1
    print(np.cross(n, o))
    print(a)
    print(np.cross(o, a))
    print(n)
    print(np.cross(a, n))
    print(o)

def zadanie_3():
    dP = np.array([[1],
                   [-2],
                   [2]])
    tsB = np.array([[2],
                    [5],
                    [0]])
    tsP = np.array([[3],
                    [-6],
                    [4]])
    RsB = SO3(np.array([[0, -1, 0],
                        [1, 0, 0],
                        [0, 0, 1]]))
    RsP = SO3(np.array([[-1, 0, 0],
                        [0, 1, 0],
                        [0, 0, -1]]))
    TsB = SE3(tsB.T) * SE3.Rz(np.pi / 2)
    TsP = SE3(tsP.T) * SE3.Ry(-np.pi)
    dS = TsP.inv() * dP
    dB = TsB * dS
    TpS = TsB * TsP.inv()
    trplot(transl(0, 0, 0), frame='B', width=1)
    trplot(TsB.A, frame='S', width=1, color='red')
    trplot(TpS.A, frame='P', width=1, color='green')
    plt.quiver(0, 0, 0, dB[0], dB[1], dB[2])
    plt.show()

# ...

# wykonywanie wybranej funkcji
if __name__ == '__main__':
    #zadanie_1()
    q1, q2, q3 = symbol('q1, q2, q3')
    RsB = np.array([[cos(q1), -sin(q1), 0],
                        [sin(q1), cos(q1), 0],
                        [0, 0, 1]])

