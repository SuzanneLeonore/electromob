import socket
import time

def send_secondary_program(script_code):
    """Envoie un programme secondaire au robot (n'interrompt pas RTDE)."""
    secondary_script = f"sec mySecondaryProg():\n{script_code}\nend\n"
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("10.2.30.60", 30002))
    sock.sendall(secondary_script.encode('utf-8'))
    sock.close()

def ouvrir_pince():
    """Commande l'ouverture de la pince OnRobot 2FG7."""
    script = "  set_tool_digital_out(0, False)"
    send_secondary_program(script)

def fermer_pince():
    """Commande la fermeture de la pince OnRobot 2FG7."""
    script = "  set_tool_digital_out(0, True)"
    send_secondary_program(script)

if __name__ == "__main__":
    print("Test d'ouverture de la pince")
    ouvrir_pince()
    time.sleep(2)

    print("Test de fermeture de la pince")
    fermer_pince()
    time.sleep(2)
