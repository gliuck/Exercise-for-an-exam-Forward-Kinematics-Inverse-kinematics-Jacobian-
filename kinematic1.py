import math
import numpy as np
from roboticstoolbox import *
import roboticstoolbox as rtb
from math import sin, cos
from spatialmath import *


# link 1
l1 = 30 #in centimetri

# link 2
l2 = 30 #in centimetri

#calcolo i dati DH con il costrutto RevoluteDH:

L1 = RevoluteDH(a=l1) # only non-zero parameters need to be specified
L2 = RevoluteDH(a=l2)

#robot = DHRobot([L1, L2], name="Robot Planare")
#print(robot)

#Joint configurations for automatic cinematic

#matrice di denavit hartenberg per il calcolo della cinematica diretta
#mi calcolo la matrice di  rotazione i rispetto alla i-1
def Forward_Kinematics(theta_i, d_i, a_i, alpha_i):
    A_i1_i = np.array (  [[cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i)],
                          [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i)],
                          [0, sin(alpha_i), cos(alpha_i), d_i],
                          [0, 0, 0, 1]])
    Rot_i1_i = SE3(A_i1_i)
    print('Ri = \n', Rot_i1_i)
    return Rot_i1_i

g1 = 10
g2 = 20

g1_rad = math.radians(g1)
g2_rad = math.radians(g2)

A_0_1 = Forward_Kinematics(g1_rad, L1.d, L1.a, L1.alpha)
A_1_2 = Forward_Kinematics(g2_rad, L2.d, L2.a, L2.alpha)

#I sdr della base sono uguali a quelli della terna0
# e che il sdr dell' ee è identica alla terna 2, abbiamo delle matrici identità  quindi
#il risultato finale sarà il medesimo:

T_0_2 = A_0_1 * A_1_2
print("Calcolo manuale della cinematica diretta: \n")
print(T_0_2)

#Metodo con le funzioni  del toolbox
#così possiamo verificare se quanto fatto è corretto

#Direct Kinematics
print("Cinematica diretta utlizzando robotics-toolbox")
robot = rtb.models.DH.Planar2()
#print("\n Parametri DH:")
#print(robot)

# robot.qz
# robot.plot(robot.qz)

print("Calcolo Automatico della Cinematica diretta ")
robot.addconfiguration("qt", [g1_rad, g2_rad])
#print(robot)

#T = robot.fkine(robot.qt)
T= robot.fkine([g1_rad, g2_rad])
print(T)


#print(g1_rad, g2_rad)

#robot.plot(robot.qtest)
