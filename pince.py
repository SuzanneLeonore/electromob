import socket
import time

class Pince:
    def __init__(self):
        self.IP_robot = "10.2.30.60"
        self.port_robot = 30002  # Connexion au robot

    def connexion(self):
        """Connexion à la pince"""
        # Connexion via socket
        self.robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot.connect((self.IP_robot, self.port_robot))

    def _action_pince(self, action):
        """Ferme ou ouvre la pince en fonction de l'action"""
        # Connecter la pince
        self.connexion()

        if action == "prise":
            self.robot.send(("set_standard_digital_out(0,True)" + "\n").encode('utf8'))  # Fermer la pince
        elif action == "lacher":
            self.robot.send(("set_standard_digital_out(0,False)" + "\n").encode('utf8'))  # Ouvrir la pince

        time.sleep(1)
        self.robot.close()

    def prise(self):
        """Fermeture de la pince"""
        self._action_pince("prise")

    def lacher(self):
        """Ouverture de la pince"""
        self._action_pince("lacher")

if __name__ == "__main__":
    pince = Pince()

    # Test ouverture et fermeture
    pince.connexion()
    pince.lacher()
    time.sleep(2)

    pince.connexion()
    pince.prise()
    time.sleep(2)
