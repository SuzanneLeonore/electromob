import numpy as np
import time
from scipy.spatial.transform import Rotation as Ro
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import Procedure_final

class Deplacement :

    

    def __init__(self, IP = "10.2.30.60"):

        self.init_pos = [-1.6491854826556605, -1.6341984907733362, 1.8493223190307617, -3.355762783681051, -1.4974659124957483, -1.5762279669391077]
        pass

    def connexion(self):
        self.robot_r = rtde_receive.RTDEReceive(self.ip)
        self.robot_c = rtde_control.RTDEControl(self.ip)

    def deconnexion(self):
        self.robot_c.disconnect()

    def prendre_Tuile(self, pos):
        self.connexion
        for i, point in enumerate(pos[:5]):
            global_point = T @ point
            pose_target = [float(x) for x in global_point[:3]] + rtde_r.getActualTCPPose()[3:]
            rtde_c.moveL(pose_target, speed=0.2, acceleration=0.2)
            time.sleep(2)
        self.deconnexion