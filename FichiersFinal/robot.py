import rtde_receive
import rtde_control
#from Transfo import create_matrice, matrice_to_pose
import numpy as np
from pince import Pince

class Robot :

    ROBOT_IP = "10.2.30.60"

    #rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    #rtde_c = rtde_control.RTDEReceiveInterface(ROBOT_IP)


    P0 = [-0.02595167,  0.8523917 ,  0.12572524]
    P1 = [0.0439653 , 0.85237297, 0.12583492]
    P2 = [-0.0249712 ,  0.88233544,  0.12582908]

    injecteur = [-0.35783121664972006, -0.2558472234496133, 0.18701865215398125]
    box = [0.000735568204164521, 0.48088659168585735, 0.0010128348377972107]

    P0 = np.array(P0)
    P1 = np.array(P1)
    P2 = np.array(P2)
    injecteur_local = np.array(list(injecteur) + [1])
    box_local= np.array(list(box) + [1])

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
    T_inv = np.linalg.inv(T)
    repere_box = T_inv @ box_local
    repere_box_1 =np.array(list(repere_box[:3])+[1])
    repere_injecteur = T_inv @ injecteur_local
    repere_injecteur_1=np.array(list(repere_injecteur[:3]) + [1])


    def connexion(self):
        """Fonction pour se connecter au robot grâce à son IP"""
        self.robot_r = rtde_receive.RTDEReceiveInterface(self.ROBOT_IP)
        self.robot_c = rtde_control.RTDEControlInterface(self.ROBOT_IP)

    def deconnexion(self): 
        """Déconnexion du robot"""
        self.robot_c.disconnect()

    def bougerJ(self, pos, speed=0.2, acceleration=0.2):
        """
        Déplacement du robot selon une pose donnée avec connexion préalable et déconnexion à la fin de l'action.

        Args:
            pos (list[float]): position à laquelle le robot doit aller.
            speed (float, optional): vitesse du déplacement.
            acceleration (float, optional): acceleration pour le déplacement.
        """
        self.connexion()
        self.robot_c.moveJ(pos, speed, acceleration)
        self.deconnexion()

    def bougerL(self, pos, speed = 0.2, acceleration =0.2) :
        self.connexion()
        global_point = Robot.T @ pos
        pose_target = [float(x) for x in global_point[:3]] + self.robot_r.getActualTCPPose()[3:]
        self.robot_c.moveL(pose_target, speed, acceleration)
        self.deconnexion()

    def deplacement_point(self, point, indice, distance):
        arrivee=point.copy()
        arrivee[indice]+=distance
        return arrivee

    def __init__(self):
        self.pose_init = [-1.6491854826556605, -1.6341984907733362, 1.8493223190307617,
                 -3.355762783681051, -1.4974659124957483, -1.5762279669391077]
        #déplacement 1cm sur X, 15cm sur Y, 10cm sur Z
        self.point1 = self.deplacement_point(self.repere_injecteur_1, 1, 0.147)
        self.point1 = self.deplacement_point(self.point1, 2, 0.15)
        self.point1 = self.deplacement_point(self.point1, 0, 0.003)
        #déplacement -15cm sur Z
        self.point2 = self.deplacement_point(self.point1, 2, -0.10)
        #déplacement -9.5cm sur X
        self.point3 = self.deplacement_point(self.point1, 0, -0.095)
        #déplacement -20 cm sur Z
        self.point4 = self.deplacement_point(self.point3, 2, -0.20)
        #déplacement -10 cm sur Z
        self.point5 = self.deplacement_point(self.point1, 2, -0.10)