import socket
import struct
import time

HOST = "10.2.30.60"  
PORT = 30003 # mettre port =3003 sinon erreur

def connect_to_robot():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    return s

def get_joint_position(sock):

    data =sock.recv(1108)

    joint_offset = 252
    joints = struct.unpack('!6d', data[joint_offset:joint_offset+48])

    return joints

def move_to_position(sock, joints):
    cmd = f"movej({joints}, a=1.0, v=0.2)\n"
    sock.send(cmd.encode('utf-8'))
    print("Envoi de la commande movej() :", joints)


def main():
    print("=== Script de Calibrage UR5 ===")
    print("1. Place manuellement le bras à la position de référence.")
    input("2. Appuie sur Entrée pour enregistrer la position actuelle...")

    sock = connect_to_robot()
    joints = get_joint_position(sock)
    print ("Position join actuelle :", joints)

    input("3. Appuie sur Entrée pour revenir à cette position avec movej()...")

    move_to_position(sock, joints)

    sock.close()
    print("Fin du script.")

if __name__ == "__main__":
    main()