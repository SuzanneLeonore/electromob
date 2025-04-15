from robot import Robot
from pince import Pince
import numpy as np
import time
"""
Nom du fichier : Pince.py
Auteur : Mattéo CAUX et Inès EL HADRI
Date : 2024-10-02
Description : Ce script contient la classe Pince et l'algorithme effectuant testants les actions de la classe
"""

temps_debut=time.time()

# création des instances de classes
robot = Robot()
pince = Pince()

pose_init = [-1.6491854826556605, -1.6341984907733362, 1.8493223190307617,
                 -3.355762783681051, -1.4974659124957483, -1.5762279669391077]

points=[
    np.array([-0.04, -0.06, 0.08, 1]),
    np.array([-0.04, 0.06, 0.08, 1]),
    np.array([-0.04, -0.12, 0.08, 1]),
    np.array([-0.04, -0.20, 0.30, 1]),
]

robot.bouger(pose_init, 0.1, 0.1)
pince.prise()
robot.bougerv2(points[0],0.1, 0.1)
robot.bougerv2(points[1],0.1, 0.1)
pince.prise()
robot.bougerv2(points[2],0.1, 0.1)
robot.bougerv2(points[3],0.1, 0.1)


temps_fin=time.time()
delta_temps=temps_fin-temps_debut
print(delta_temps)