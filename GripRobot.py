import socket
import time
import rtde_receive
import rtde_control
import numpy as np

class Pince :
    def __init__(self):
        self.IP_robot = "10.2.30.60"
        self.port_dashboard = 29999  # Pour la connexion via socket à l'IHM
        self.port_robot = 30002  # Pour la connexion via socket au robot lui-même

    def connexion(self):
        """Connexion à la Pince"""
        # socket permet d'envoyer des commandes script à faire exécuter à l'IHM ou au robot
        self.robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot.connect((self.IP_robot, self.port_robot))
        self.dashboard.connect((self.IP_robot, self.port_dashboard))

        self.dashboard.send(("load gripper_open.urp" + "\n").encode('utf-8'))

    def _action_pince(self, action):
        """
        Fermer ou Ouvrir la pince en fonction de l'action demandée.

        Args:
            action (string): "prise" ou "lacher"
        """
        # connecter la pince
        self.connexion()

        # Changement de valeur de la sortie suivant le mouvement de pince à effectuer



        if action == "prise": 
            self.robot.send(("set_standard_digital_out(0,True)" + "\n").encode('utf8'))
        elif action == "lacher":
            self.robot.send(("set_standard_digital_out(0,False)" + "\n").encode('utf8'))

        time.sleep(1)
        self.dashboard.send(("stop" + "\n").encode('utf8'))  # On arrête à nouveau le programme local

        # deconnecter la pince
        self.robot.close()
        self.dashboard.close()
    
    def prise(self):
        """Fermeture de la pince"""
        self._action_pince("prise")

    def lacher(self):
        """Ouverture de la pince"""
        self._action_pince("lacher")

if __name__=="__main__":
    pince = Pince()

    #test de lacher
    pince.connexion()
    pince.lacher()

    # au tour du robot

    # test de prise
    pince.connexion()
    pince.prise()

    # au tour du robot



  




    

    





