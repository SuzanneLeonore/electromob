import numpy as np
import time
from scipy.spatial.transform import Rotation as Ro
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
#from GripRobot import Pince


#maPince=Pince()
'''
import socket

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

'''

ROBOT_IP = "10.2.30.60"

rtde_c = RTDEControl(ROBOT_IP)
rtde_r = RTDEReceive(ROBOT_IP)

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

'''
input("prendre point de l'injecteur")
q_current = rtde_r.getActualTCPPose()[:3]
print(q_current)

input("prendre point de la box")
q_current = rtde_r.getActualTCPPose()[:3]
print(q_current)

q_current = rtde_r.getActualQ()
print(q_current)


pose_init=[-1.6491854826556605, -1.6341984907733362, 1.8493223190307617, -3.355762783681051, -1.4974659124957483, -1.5762279669391077]
rtde_c.moveJ(pose_init, speed=0.2, acceleration=0.2)
'''
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


print("\n=== Repère local défini ===")
print("Origine :", P0)
print("Axes :\nX:", x_axis, "\nY:", y_axis, "\nZ:", z_axis)
input("va faire un déplacement")


def cheminTuile():
    print(">> Début de cheminTuile()")

    pose_init = [-1.6491854826556605, -1.6341984907733362, 1.8493223190307617,
                 -3.355762783681051, -1.4974659124957483, -1.5762279669391077]
    rtde_c.moveJ(pose_init, speed=0.2, acceleration=0.2)
    print(">> MoveJ init OK")

    #ouvrir_pince()
    #print(">> Pince ouverte")

    for i, point in enumerate(points):
        #if i == 1:
            #fermer_pince()
            #print(">> Pince fermée (prise tuile)")

        global_point = T @ point
        pose_target = [float(x) for x in global_point[:3]] + rtde_r.getActualTCPPose()[3:]
        rtde_c.moveL(pose_target, speed=0.2, acceleration=0.2)
        time.sleep(2)

    for i, point in enumerate(joints):
        rtde_c.moveJ(point, speed=0.2, acceleration=0.2)
        time.sleep(2)

    for i, point in enumerate(points2):
        global_point = T @ point
        pose_target = [float(x) for x in global_point[:3]] + rtde_r.getActualTCPPose()[3:]
        rtde_c.moveL(pose_target, speed=0.2, acceleration=0.2)

        #if i == 1 or i == 9:
            #ouvrir_pince()
            #print(f">> Pince ouverte à l'étape {i}")

        #if i == 4 or i == 12:
            #fermer_pince()
            #print(f">> Pince fermée à l'étape {i}")

        time.sleep(2)

    for i, point in enumerate(joints2):
        rtde_c.moveJ(point, speed=0.2, acceleration=0.2)
        time.sleep(2)

    print(">> Fin de cheminTuile()")


def deplacement_point(point, indice, distance):
    arrivee=point.copy()
    arrivee[indice]+=distance
    return arrivee

points=[
    np.array([-0.04, -0.06, 0.08, 1]),
    np.array([-0.04, 0.06, 0.08, 1]),
    np.array([-0.04, -0.12, 0.08, 1]),
    np.array([-0.04, -0.20, 0.30, 1]),
    #np.array(repere_injecteur)
]
#déplacement 1cm sur X, 15cm sur Y, 10cm sur Z
point1 = deplacement_point(repere_injecteur, 1, 0.15)
point1 = deplacement_point(point1, 2, 0.10)
point1 = deplacement_point(point1, 0, 0.01)
#déplacement -15cm sur Z
point2 = deplacement_point(point1, 2, -0.15)
#déplacement -9.5cm sur X
point3 = deplacement_point(point1, 0, -0.095)
#déplacement -20 cm sur Z
point4 = deplacement_point(point3, 2, -0.20)
#déplacement -10 cm sur Z
point5 = deplacement_point(point1, 2, -0.10)

'''
point1[1] += 0.15
point1[2] += 0.10
point1[0] +=-0.01
point2=point1.copy()
point2[2] += -0.15
point3=point1.copy()
point3[0]+= -0.095
point4=point3.copy()
point4[2]-=0.20
point5=point1.copy()
point5[2]+=-0.10
'''
points2=[
    np.array(point1),
    np.array(point2),
    np.array(point1),
    np.array(point3),
    np.array(point4),
    np.array(point3),
    np.array(point1),
    np.array(point5),
    np.array(point1),
    np.array(point3),
    np.array(point4),
    np.array(point3),
    np.array(point1),
    np.array(point2),
    np.array(point1),
]


joints=[
    [-1.6755712668048304, -1.4491103331195276, 0.8367433547973633, -0.9699614683734339, -1.4714487234698694, -1.5762398878680628],
    [-0.4743412176715296, -1.5091918150531214, 1.348893642425537, -1.3945730368243616, -1.4682758490191858, -2.0456507841693323],
    #[-0.45730573335756475, -1.5136826674090784, 1.6165494918823242, -1.6594861189471644, -1.468060318623678, -2.0286367575274866],
    #ouverture pince
    #[-0.588726822529928, -1.6191085020648401, 2.0765199661254883, -2.000685993825094, -1.4708860556231897, -2.160619084035055],
    #serre pince 
    #[-0.5981023947345179, -1.632378403340475, 2.0899696350097656, -1.9999058882342737, -1.4710419813739222, -2.1699631849872034],
    #[-0.4743412176715296, -1.5091918150531214, 1.348893642425537, -1.3945730368243616, -1.4682758490191858, -2.0456507841693323]
]

joints2=[
    [-1.5707710425006312, -1.9037888685809534, 1.8204197883605957, -1.5371840635882776, -1.4706586042987269, -1.5850275198565882]
]



#robot tourne de X degrees

def rotate_UR5_around_local_Z(degrees):
    pose = rtde_r.getActualTCPPose()
    pos = np.array(pose[:3])
    rotvec = np.array(pose[3:])

    R_current = Ro.from_rotvec(rotvec).as_matrix()
    angle_rad = np.radians(degrees)
    Rz = Ro.from_euler('z', angle_rad).as_matrix()

    R_new = Rz @ R_current  # rotation Z * orientation actuelle
    new_rotvec = Ro.from_matrix(R_new).as_rotvec()

    pose_target = list(pos) + list(new_rotvec)
    print("→ Rotation autour de Z (global) de", degrees, "°")
    print("Pose cible :", pose_target)

    rtde_c.moveL(pose_target, speed=0.2, acceleration=0.5)


if __name__ == "__main__":
    cheminTuile()

    while rtde_c.isProgramRunning():
        time.sleep(0.1)

    rtde_c.stopScript()
    print("Mouvement terminé.")

