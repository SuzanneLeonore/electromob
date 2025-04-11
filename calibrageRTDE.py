import numpy as np
import time
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

ROBOT_IP = "10.2.30.60"

rtde_c = RTDEControl(ROBOT_IP)
rtde_r = RTDEReceive(ROBOT_IP)

def get_position():
    return np.array(rtde_r.getActualTCPPose()[:3])

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
# déplacement : 1cm en X
local_point = np.array([0.01, 0.0, 0.0, 1])  # [X, Y, Z, 1]

global_point = T @ local_point
pose_target = [float(x) for x in global_point[:3]] + rtde_r.getActualTCPPose()[3:]


print("\nDéplacement vers :", pose_target)
rtde_c.moveL(pose_target, speed=0.25, acceleration=0.5)

# Attendre la fin du mouvement
while rtde_c.isProgramRunning():
    time.sleep(0.1)

rtde_c.stopScript()
print("Mouvement terminé.")
