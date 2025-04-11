import numpy as np
import time
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

ROBOT_IP = "10.2.30.60"

rtde_c = RTDEControl(ROBOT_IP)
rtde_r = RTDEReceive(ROBOT_IP)

def get_position():
    return np.array(rtde_r.getActualTCPPose()[:3])


print("### Calibrage repère local avec RTDE ###")

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