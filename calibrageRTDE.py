import numpy as np
import time
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

ROBOT_IP = "10.2.30.60"

rtde_c = RTDEControl(ROBOT_IP)
rtde_r = RTDEReceive(ROBOT_IP)
P0 = [-0.02595167,  0.8523917 ,  0.12572524]
P1 = [0.0439653 , 0.85237297, 0.12583492]
P2 = [-0.0249712 ,  0.88233544,  0.12582908]

P0 = np.array(P0)
P1 = np.array(P1)
P2 = np.array(P2)


def get_position():
    return np.array(rtde_r.getActualTCPPose()[:3])

print(rtde_r.getActualTCPPose())

'''
print(" Calibrage repère local avec RTDE ")

input("Place le robot à l'ORIGINE du repère local (P0), puis appuie sur Entrée...")
P0 = get_position()

input("Place le robot sur l’AXE X local (P1), puis appuie sur Entrée...")
P1 = get_position()

input("Place le robot sur l’AXE Y local (P2), puis appuie sur Entrée...")
P2 = get_position()

print("Points capturés.")
print("P0 :", P0)
print("P1 :", P1)
print("P2 :", P2)
'''
 
# Calcul du repère local (rotation + origine)
x_axis = P1 - P0
x_axis /= np.linalg.norm(x_axis)

temp_y = P2 - P0
z_axis = np.cross(x_axis, temp_y)
z_axis /= np.linalg.norm(z_axis)

y_axis = np.cross(z_axis, x_axis)

# Matrice de rotation (repère local → global)
R = np.column_stack((x_axis, y_axis, z_axis))

T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = P0

print("\n=== Repère local défini ===")
print("Origine :", P0)
print("Axes :\nX:", x_axis, "\nY:", y_axis, "\nZ:", z_axis)
input("va faire un déplacement")

points=[
    np.array([-0.04, -0.06, 0.08, 1]),
    np.array([-0.04, 0.06, 0.08, 1]),
    np.array([-0.04, -0.12, 0.08, 1]) 
]

for i, point in enumerate(points):
    global_point = T @ point
    pose_target = [float(x) for x in global_point[:3]] + rtde_r.getActualTCPPose()[3:]
    rtde_c.moveL(pose_target, speed=0.1, acceleration=0.1)
    time.sleep(10)  

'''
local_point1 = np.array([-0.04, -0.06, -0.08, 1]) 
#ouvrir pince
local_point2 = np.array([-0.04, 0.06, 0.08, 1])
#fermer pince
local_point3 = np.array([-0.04, -0.12, -0.08, 1]) 

global_point = T @ local_point1

pose_target = [float(x) for x in global_point[:3]] + rtde_r.getActualTCPPose()[3:]
pose_new=P0

print("\n[DEBUG] Vérification de la position actuelle :")
print("TCP position :", rtde_r.getActualTCPPose())
print("Target pose :", pose_target)
print("Robot mode :", rtde_r.getRobotMode())      # 7 = RUNNING
print("Safety mode :", rtde_r.getSafetyMode())

print("\nDéplacement vers :", pose_target)
rtde_c.moveL(pose_target, speed=0.1, acceleration=0.1)

# Attendre la fin du mouvement
while rtde_c.isProgramRunning():
    time.sleep(0.1)
'''

rtde_c.stopScript()
print("Mouvement terminé.")
