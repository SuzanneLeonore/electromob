import socket
import struct
import time

HOST = "10.2.30.60"  
PORT = 30003 # mettre port =3003 sinon erreur
point_calibrage=[]
delta1=[0.12,-0.07,0.14,-0.33,0.001,0.0]
delta2=[-0.02, -0.08, 0.14, 0.06, -0.03,-0.004]
delta=[0.323, -0.035, -0.149, 0.186, 0.323, 0.0]
#delta1=[0,0.00,-0.0,0.0,0.01,0.0]
point_passage=[]

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

def variation_position1(sock,i):
    clone = list(point_calibrage[i])
    clone = [c + d for c, d in zip(clone, delta)]
    cmd = f"movej({clone}, a=1.0, v=0.2)\n"
    sock.send(cmd.encode('utf-8'))

def variation_position2(sock,i):
    clone = list(point_calibrage[i])
    clone = [c + d for c, d in zip(clone, delta2)]
    cmd = f"movej({clone}, a=1.0, v=0.2)\n"
    sock.send(cmd.encode('utf-8'))

def main():
    print("=== Script de Calibrage UR5 ===")
    print("1. Place manuellement le bras à la position de référence n°1.")
    input("2. Appuie sur Entrée pour enregistrer la position actuelle...")

    sock = connect_to_robot()
    joints1 = get_joint_position(sock)
    point_calibrage.append(joints1)
    '''
    print ("Position join actuelle :", joints1)
    print("3. Place manuellement le bras à la position de référence n°2.")
    input("4. Appuie sur Entrée pour enregistrer la position actuelle...")
    joints2 = get_joint_position(sock)
    point_calibrage.append(joints2)

    print("5. Place manuellement le bras à la position de référence n°3.")
    input("6. Appuie sur Entrée pour enregistrer la position actuelle...")
    joints3 = get_joint_position(sock)
    point_calibrage.append(joints3)
    '''
    input("7. Appuie sur Entrée pour prendre la tuile...")
    
    variation_position1(sock,0)
    time.sleep(0.3)
    #point_calibrage[0] = get_joint_position(sock)
    #variation_position2(sock,0)

    #move_to_position(sock, joints)

    sock.close()
    print("Fin du script.")

if __name__ == "__main__":
    main()