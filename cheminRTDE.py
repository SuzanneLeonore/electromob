from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import numpy as np

ROBOT_IP = "10.2.30.60"

point_calibrage=[] #utilisation du convoyeur
points = [
    [-1.58, -1.8, 1.63, -1.31, -1.6, 1.73],
    [-1.58, -1.8, 1.63, -2.90, -1.6, 1.73],
]


# Initialisation des interfaces
rtde_c = RTDEControl(ROBOT_IP)
rtde_r = RTDEReceive(ROBOT_IP)

# Lecture de la position TCP actuelle

tcp_pos = rtde_r.getActualTCPPose()  # [x, y, z, rx, ry, rz]
point_calibrage.append(tcp_pos)
print("Position actuelle TCP :", tcp_pos)

rtde_c.moveJ(points[0])
print('mouvement1')
rtde_c.moveJ(points[1])
print('mouvement2')

'''
#création du repère 
input("placer le bras sur l'origine")
tcp_pos = rtde_r.getActualTCPPose()  
point_calibrage.append(tcp_pos)
print("Position actuelle TCP :", tcp_pos)

input("placer le bras sur l'axe des X")
tcp_pos = rtde_r.getActualTCPPose()  
point_calibrage.append(tcp_pos)

input("placer le bras sur l'axe des Y")
tcp_pos = rtde_r.getActualTCPPose() 
point_calibrage.append(tcp_pos)

P0 = np.array(point_calibrage[0][:3])
P1 = np.array(point_calibrage[1][:3])
P2 = np.array(point_calibrage[2][:3])

x_axis = P1 - P0
x_axis /= np.linalg.norm(x_axis)

y_temp = P2 - P0
z_axis = np.cross(x_axis, y_temp)
z_axis /= np.linalg.norm(z_axis)

y_axis = np.cross(z_axis, x_axis)

# Matrice de rotation
R = np.column_stack((x_axis, y_axis, z_axis))

T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = P0

local_target = np.array([0.1, 0.0, 0.0, 1])  # ex: 10 cm en X
global_target = T @ local_target  # multiplication matrice

pose_to_send = list(global_target[:3]) + list(rtde_r.getActualTCPPose()[3:])
rtde_c.moveL(pose_to_send, speed=0.25, acceleration=0.5)


# Déplacement vers un point relatif (ex: +1 cm en Z)
mouvement=[]
target_pose = tcp_pos.copy()
target_pose[2] += 0.01  # monter de 1 cm

# Envoyer la commande de mouvement cartésien
rtde_c.moveL(target_pose, speed=0.25, acceleration=0.5)

while rtde_c.isProgramRunning():
    time.sleep(0.1)

# Arrêter proprement
rtde_c.stopScript()
'''