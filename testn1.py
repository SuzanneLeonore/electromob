from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time

rtde_c = RTDEControlInterface("10.2.30.60")
rtde_r = RTDEReceiveInterface("10.2.30.60")

pose_init = [-1.6, -1.6, 1.8, -3.3, -1.4, -1.5]
rtde_c.moveJ(pose_init, speed=0.3, acceleration=0.5)

while rtde_c.isProgramRunning():
    time.sleep(0.1)

print("✔ Mouvement terminé.")
rtde_c.stopScript()
