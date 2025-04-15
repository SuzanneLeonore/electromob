"""
Nom du fichier : ProjetUR5.py
Auteur : Suzanne GIRARD-JOLLET et Alban CASELLA
Date : 2025-04-15
Description : Ce script réalise les différentes étapes du projet, de la prise de la tuile depuis la convoyeur au rangemenent de la tuile dans la boite
"""

# import
import cheminTuile1
from Pince import Pince
import time 

import numpy as np


temps_debut=time.time()

# création des instances de classes
#robot = Robot()
pince = Pince()

# déplacement du robot à sa position initiale
cheminTuile1.cheminTuile()



# calcul du temps d'éxécution
temps_fin=time.time()
delta_temps=temps_fin-temps_debut
print(delta_temps)