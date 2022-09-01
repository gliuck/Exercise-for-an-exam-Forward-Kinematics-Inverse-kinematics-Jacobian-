import math
from math import pi, sin, cos
import numpy as np
import roboticstoolbox as rtb


#Parametri  Robot
l1 = 30
l2 = 30

L1 = rtb.RevoluteDH(a=l1)
L2 = rtb.RevoluteDH(a=l1)
jrobot = rtb.DHRobot([L1,L2])
print(jrobot)

#Vettore delle variabili di giunto
g1 = 10
g2 = 20
g1_rad = math.radians(g1)
g2_rad = math.radians(g2)
q = np.array([g1_rad, g2_rad])

#Calcolo jacobian geometrico manipolatore a due bracci
print(jrobot.jacob0(q))

#Calcolo Jacobiano geometrico manipolatore generico

grobot = rtb.models.DH.UR5()
#print(grobot)
g_test1 = 40
g_test2 = 11
g_test3 = 10
g_test4 = 66
g_test5 = 87
g_test6 = 12
gt1_rad = math.radians(g_test1)
gt2_rad = math.radians(g_test2)
gt3_rad = math.radians(g_test3)
gt4_rad = math.radians(g_test4)
gt5_rad = math.radians(g_test5)
gt6_rad = math.radians(g_test6)
qt = np.array([gt1_rad, gt2_rad, gt3_rad, gt4_rad, gt5_rad, gt6_rad])


##Risultati Jacobiano Geoemtrico

#Calcolo jacobian geometrico manipolatore a due bracci
#print(jrobot.jacob0(q))

#Calcolo jacobiano geoemtrico manipolatore generico
#print(grobot.jacob0(qt))
#grobot.plot(grobot.qr)

# ur5 = rtb.models.UR5()
# ur5.plot(ur5.qz, block=True)