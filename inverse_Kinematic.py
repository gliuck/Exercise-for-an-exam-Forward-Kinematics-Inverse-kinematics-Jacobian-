import math
import numpy as np
from roboticstoolbox import *
import roboticstoolbox as rtb
from math import sin, cos
from spatialmath import *

#Dati Manipolatore

#link1
l1 = 30
#link2
l2 = 30

print("Calcolo manuale della cinematica diretta \n")

#Posizone End-Effectot
Px = 34
Py = 46

Px_rad = math.radians(34)
Py_rad = math.radians(46)

#print(Px_rad, Py_rad)

r_2 = Px**2 + Py**2

#Verifico che Px e Py siano corretti per il nostro Workspace
print("Se c2 compreso tra -1 e 1 Px e Py sono corretti per il Workspace\n")

c2 = (-l1**2 - l2**2 + r_2) / (2*l1*l2)
print("c2 = ",c2, "\n")

if c2 <= 1 and c2 >= -1:
    #Calcolo s2
    s2 = math.sqrt(1- pow(c2, 2))

    #Posso ora ricavarmi le mie variabili di giunti theta1 theta2

    theta_2 = math.atan2(s2, c2)
    theta_1 = math.atan2(Py, Px) - math.atan2(l2*s2, l1+l2*c2)

    t1 = math.degrees(theta_1)
    t2 = math.degrees(theta_2)


print("Le variabili di giunto della cinematica inversa  calcolata manualmente sono:\n")
print("theta1 = ",t1, "theta2 = ", t2 , "\n")

robot = rtb.models.DH.Planar2()

#vado a inserire le variabili di giunto appena calcolate
T1= robot.fkine([theta_1, theta_2])

#calcolo l'inversa  passadogli la matrice di trasformazione omogenea
sol = robot.ikine_LM(T1)

sol1 = sol[0][0]
sol2 = sol[0][1]

th1 = math.degrees(sol1)
th2 = math.degrees(sol2)

print("Le variabili di giunto della cinematica inversa calcolata Automaticamente sono: \n")
print("theta1 =", th1,"theta2 =", th2)
