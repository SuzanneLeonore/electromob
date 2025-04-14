import json
import paho.mqtt.client as mqtt
import time
import threading
import numpy as np 
import rtde_receive
import rtde_control
import dashboard_client
import math
import cv2 as cv
import time
import cheminTuile1

# Variables globales pour partager les données
convoyeur_data = [0] * 2  # Pour capteurs_convoyeur
bac_data = [0] * 5        # Pour capteurs_bac

# Variable globale "global_out" accessible de partout dans le programme,
# ainsi qu'un verrou pour protéger l'accès concurrent
global_out = None
out_lock = threading.Lock()

##########################################
# Partie 1 : MQTT
##########################################

# Paramètres MQTT
MQTT_BROKER = "10.2.30.162"
MQTT_PORT = 1883
TOPIC_B = "capteurs_bac/etat"
TOPIC_C = "capteurs_convoyeur/etat"

def on_connect(client, userdata, flags, rc):
    print("Connecté au broker MQTT avec le code de retour", rc)
    client.subscribe(TOPIC_B)
    client.subscribe(TOPIC_C)

def on_message(client, userdata, msg):
    global convoyeur_data, bac_data, global_out
    try:
        # Décoder le message JSON
        data = json.loads(msg.payload.decode('utf-8'))
        
        # Mettre à jour les données en fonction du topic
        if msg.topic == TOPIC_C:  # capteurs_convoyeur/etat (2 pins)
            convoyeur_data = [data["pin1"], data["pin2"]]
        elif msg.topic == TOPIC_B:  # capteurs_bac/etat (5 pins)
            bac_data = [data[f"pin{i+1}"] for i in range(5)]
        
        # Mettre à jour la variable globale "global_out" de façon sécurisée
        with out_lock:  # Début de la section critique
            global_out = [convoyeur_data, bac_data]
        
        print("Données MQTT reçues :", global_out)
    
    except Exception as e:
        print("Erreur lors du traitement du message :", e)

def mqtt_client_thread():
    client = mqtt.Client("PythonClient")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    
    # Boucle bloquante dans un thread séparé
    client.loop_forever()

##########################################
# Partie 2 : Contrôle du robot via rtde
##########################################

def robot_control_thread():
    # Adresse IP du robot
    robot_ip = "10.2.30.60"
    
    # Initialisation des interfaces RTDE pour le contrôle et la réception de données
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    
    # Définition des points de destination (vecteurs de joints)
    # Vous pouvez définir ici la séquence complète des mouvements à exécuter
    points = [
        #[-1.58, -1.8,  1.63, -1.31, -1.6, 1.73],
        #[-1.58, -1.8,  1.63, -2.90, -1.6, 1.73],
        #[-1.58, -1.29, 2.39, -4.19, -1.62, 1.67],
        #[-1.58, -1.21, 2.33, -4.19, -1.62, 1.67],
        [-1.58, -1.15, 2.22, -4.13, -1.62, 1.67],
        #[-1.58, -1.25, 2.42, -4.23, -1.62, 1.67],
        #[-1.58, -1.81, 2.99, -3.24, -1.62, 1.67]
    ]
    
    # Variable d'état pour s'assurer qu'on ne lance la séquence qu'une fois par activation.
    triggered = False
    
    try:
        while True:
            # Lecture sécurisée de la variable globale
            with out_lock:
                current_data = global_out
            
            # On vérifie que les données sont disponibles
            if current_data is not None:
                # Si le capteur (par exemple global_out[0][0]) est à 1 et qu'on n'a pas encore déclenché la séquence
                if current_data[0][0] == 1 and not triggered:
                    triggered = True
                    print("Déclenchement de la séquence de mouvements.")
                    
                    cheminTuile1.cheminTuile()
                    # Exécuter la séquence des mouvements
                    for i, target_point in enumerate(points):
                        print(f"Envoi du mouvement {i+1} au robot via RTDE.")
                         #Utilisation de moveJ avec speed et acceleration (adapter les valeurs selon vos besoins)
                        rtde_c.moveJ(target_point, speed=0.2, acceleration=0.5)
                        
                        # Attendre un délai pour permettre l'exécution du mouvement (pour la démonstration)
                        time.sleep(4)
                    
                    print("Séquence de mouvements terminée.")
                
                # Réinitialiser l'état dès que la valeur revient à 0 afin de pouvoir déclencher de nouveau
                elif current_data[0][0] == 0 and triggered:
                    triggered = False
                    print("Réinitialisation de l'état de déclenchement.")
            
            # Petite pause pour éviter une boucle trop gourmande en CPU
            time.sleep(0.1)
    
    except Exception as e:
        print("Erreur lors de la communication avec le robot via RTDE :", e)
    
    finally:
        # Optionnel : arrête le programme ou libère la connexion si besoin
        rtde_c.stopScript()  # Arrête le script sur le robot (si nécessaire)
        print("Contrôle via RTDE terminé")



##########################################
# Démarrage des threads
##########################################

if __name__ == "__main__":
    # Lancer le thread du client MQTT
    mqtt_thread = threading.Thread(target=mqtt_client_thread, daemon=True)
    mqtt_thread.start()
    
    # Laisser un peu de temps pour la connexion MQTT
    time.sleep(1)
    
    # Lancer le thread de contrôle du robot via RTDE
    robot_thread = threading.Thread(target=robot_control_thread, daemon=True)
    robot_thread.start()
    
    robot_thread.join()  # Attendre la fin du thread robot si non-infini
    
    # Garder le programme actif pour le thread MQTT
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Arrêt du programme.")

