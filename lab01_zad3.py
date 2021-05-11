import matplotlib.pyplot as plt
import numpy as np
from spatialmath import *
from spatialmath.base import *

#Dane
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
trplot( transl(0,0,0), frame='B', width=1)
trplot( TsB.A, frame='S', width=1, color='red')
trplot( TpS.A, frame='P', width=1, color='green')
plt.quiver( 0, 0, 0, dB[0], dB[1], dB[2])
plt.show()