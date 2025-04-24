import cv2
import numpy as np
import pyrealsense2 as rs
import time
import math


# Pour le contrôle du robot UR5
try:
    import rtde_control
    import rtde_receive
    UR5_AVAILABLE = True
except ImportError:
    print("AVERTISSEMENT: Module rtde_control non disponible, mode simulation seulement.")
    UR5_AVAILABLE = False

class ArucoTracker:
    def __init__(self):
        # --- Configuration utilisateur ---
        self.NUM_MARKERS = 2  # nombre de marqueurs à utiliser pour le calcul du milieu

        self.T_cam_to_base = np.eye(4)

        # --- Facteur de correction d'échelle ---
        # 10cm dans le code correspondent à 6.5cm dans la réalité, donc facteur correctif = 10/6.5
        self.scale_correction = 10/6.5  # ≈ 1.54
                
        # --- Initialisation RealSense ---
        self.pipeline = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.cfg)

        self.align = rs.align(rs.stream.color)
        self.depth_intrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        
        # --- Paramètres caméra pour pose ArUco ---
        self.camera_matrix = np.array([[self.depth_intrin.fx, 0, self.depth_intrin.ppx],
                                       [0, self.depth_intrin.fy, self.depth_intrin.ppy],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)  # Approx. sans distorsion
        
        # --- Paramètres ArUco ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.marker_size = 0.05  # Taille du marqueur en mètres (à ajuster)
        
        # --- Variables pour le suivi ---
        self.mid_base = None  # Point milieu dans les coordonnées du robot
        self.aruco_axes = None  # Axes X, Y, Z des ArUco dans le repère robot
        
        # --- Initialisation UR5 ---
        self.ur5_control = None
        self.ur5_receive = None


        # --- Variables pour le calcul de moyenne ---
        self.position_history = []  # Historique des positions du milieu
        self.axes_history = []  # Historique des axes détectés
        self.avg_mid_base = None  # Position moyenne du milieu
        self.avg_axes = None  # Axes moyens
        self.detection_count = 0  # Compteur de détections valides
        self.max_detections = 100  # Nombre de détections pour calculer la moyenne
        self.calibration_done = False  # Indique si la calibration est terminée
        
        if UR5_AVAILABLE:
            try:
                robot_ip = "10.2.30.60"
                self.ur5_control = rtde_control.RTDEControlInterface(robot_ip)
                self.ur5_receive = rtde_receive.RTDEReceiveInterface(robot_ip)
                print("Robot UR5 connecté avec succès.")
                
                # === Mise à jour dynamique de T_cam_to_base ===
                # 1) Récupérer pose TCP (X, Y, Z, Rx, Ry, Rz)
                tcp = self.ur5_receive.getActualTCPPose()
                
                # 2) Construire la matrice base→TCP
                T_base_tcp = np.eye(4)
                T_base_tcp[:3, 3] = tcp[:3]
                # Rx,Ry,Rz est déjà un vecteur de rotation (Rodrigues)
                R_base_tcp, _ = cv2.Rodrigues(np.array(tcp[3:6], dtype=float))
                T_base_tcp[:3, :3] = R_base_tcp
                
                # 3) Définir l’offset TCP→Caméra
                T_tcp_cam = np.eye(4)
                # 0.103 m devant (X), 0 m latéral (Y), -0.14 m dessous (Z)
                T_tcp_cam[:3, 3] = [0.103, 0.0, -0.14]
                
                # 4) Composer pour obtenir base→Caméra
                #    C’est justement la transform qu’on applique aux points
                self.T_cam_to_base = T_base_tcp @ T_tcp_cam

                print("Transform caméra→base calculée dynamiquement.")
            except Exception as e:
                print(f"Erreur connexion ou calcul transform UR5: {e}")
                # MAIS on laisse la valeur identité en place pour simulation

            
            except Exception as e:
                print(f"Erreur connexion ou calcul transform UR5: {e}")
                self.ur5_control = None
                self.ur5_receive = None
    
    def deproject(self, px, py, depth):
        """Convertit des coordonnées pixel en point 3D dans le repère caméra"""
        return np.array(rs.rs2_deproject_pixel_to_point(self.depth_intrin, [px, py], depth), dtype=float)
    
    def detect_aruco_with_pose(self, img, depth_frame):
        """Détecte les marqueurs ArUco et calcule leur pose 3D"""
        # Détection des ArUco
        corners, ids, _ = self.detector.detectMarkers(img)
        
        if ids is None or len(ids) < self.NUM_MARKERS:
            return img, None, None, None
        
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        
        # Récupération des IDs présents et tri croissant
        present = sorted(int(i[0]) for i in ids)
        # On prend les NUM_MARKERS premiers
        selected_ids = present[:self.NUM_MARKERS]
        
        pts3d = []
        rotation_matrices = []
        valid_markers = []
        
        # Pour chaque marqueur sélectionné, estimer sa pose
        for i, (corner, mid) in enumerate(zip(corners, ids)):
            mid_id = int(mid[0])
            if mid_id in selected_ids:
                # Estimer la pose du marqueur
                try:
                    # Estimation de pose
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corner], self.marker_size, self.camera_matrix, self.dist_coeffs
                    )
                    
                    # Dessiner les axes
                    cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size/2)
                    
                    # Convertir le vecteur de rotation en matrice de rotation
                    rot_matrix, _ = cv2.Rodrigues(rvec[0])
                    
                    # Calculer le centre du marqueur
                    c = corner[0]
                    cx, cy = int(c[:, 0].mean()), int(c[:, 1].mean())
                    
                    # Patch 5×5 pour lisser la profondeur
                    depths = []
                    for dx in (-2, -1, 0, 1, 2):
                        for dy in (-2, -1, 0, 1, 2):
                            d = depth_frame.get_distance(cx + dx, cy + dy)
                            if d > 0:
                                depths.append(d)
                    if not depths:
                        continue
                    depth = float(np.median(depths))
                    
                    # Deprojection pour vérification
                    p_cam = self.deproject(cx, cy, depth)
                    
                    # Correction: utiliser la profondeur mesurée pour affiner la position
                    # tvec[0][2] = depth  # Optionnel: remplacer Z par la profondeur mesurée
                    
                    pts3d.append(p_cam)
                    rotation_matrices.append(rot_matrix)
                    valid_markers.append(mid_id)
                    
                    # Marquage visuel
                    cv2.circle(img, (cx, cy), 4, (255, 0, 0), -1)
                    cv2.putText(img, f"ID={mid_id}", (cx + 5, cy - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                except Exception as e:
                    print(f"Erreur estimation pose pour marqueur {mid_id}: {e}")
                    continue
        
        # Si on a assez de marqueurs, calculer le milieu et construire les axes
        if len(valid_markers) == self.NUM_MARKERS and len(rotation_matrices) == self.NUM_MARKERS:
            # Point milieu caméra
            mid_cam = np.mean(pts3d, axis=0)
            
            # Extraire les axes des marqueurs 
            # Ces axes sont dans le repère caméra
            x_axes = [rot[:, 0] for rot in rotation_matrices]  # Axe X de chaque marqueur
            y_axes = [rot[:, 1] for rot in rotation_matrices]  # Axe Y de chaque marqueur
            z_axes = [rot[:, 2] for rot in rotation_matrices]  # Axe Z de chaque marqueur
            
            # Moyenner les directions des axes
            x_axis = np.mean(x_axes, axis=0)
            y_axis = np.mean(y_axes, axis=0)
            z_axis = np.mean(z_axes, axis=0)
            
            # Orthonormaliser les axes (pour garantir un repère orthogonal)
            # Cette étape est importante car la moyenne peut ne pas donner un repère orthogonal
            z_axis = z_axis / np.linalg.norm(z_axis)  # Normaliser Z
            
            # Corriger Y pour être perpendiculaire à Z
            y_axis = y_axis - np.dot(y_axis, z_axis) * z_axis
            y_axis = y_axis / np.linalg.norm(y_axis)  # Normaliser Y
            
            # X doit être perpendiculaire à Y et Z (produit vectoriel)
            x_axis = np.cross(y_axis, z_axis)  # Déjà normalisé car Y et Z sont orthonormaux
            
            # Construction de la matrice de rotation complète dans le repère caméra
            R_aruco_cam = np.column_stack([x_axis, y_axis, z_axis])
            
            # Transformation caméra→robot du point milieu
            hom = np.ones(4)
            hom[:3] = mid_cam
            mid_base = (self.T_cam_to_base @ hom)[:3]
            
            # Transformation des axes dans le repère robot
            # On ne transforme que la partie rotation, pas la translation
            R_cam_to_base = self.T_cam_to_base[:3, :3]
            
            # Transformer chaque axe dans le repère robot
            x_axis_base = R_cam_to_base @ x_axis
            y_axis_base = R_cam_to_base @ y_axis
            z_axis_base = R_cam_to_base @ z_axis
            
            # Stocker les axes pour une utilisation future
            self.aruco_axes = {
                'x': x_axis_base,
                'y': y_axis_base,
                'z': z_axis_base
            }
            
            # Texte 3D avec liste d'IDs
            txt = f"Milieu base({'+'.join(map(str, valid_markers))}): " + \
                  f"[{mid_base[0]:.3f}, {mid_base[1]:.3f}, {mid_base[2]:.3f}]"
            cv2.putText(img, txt, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Projection dans l'image
            proj = rs.rs2_project_point_to_pixel(self.depth_intrin, mid_cam.tolist())
            u, v = int(proj[0]), int(proj[1])
            cv2.circle(img, (u, v), 6, (0, 255, 255), -1)
            
            # Dessiner les axes projetés du repère ArUco au point milieu (option visuelle)
            axis_length = 0.1  # Longueur des axes en mètres
            
            # Créer un vecteur de rotation qui représente l'orientation des axes
            rot_vec_aruco, _ = cv2.Rodrigues(R_aruco_cam)
            
            # Dessiner les axes du repère ArUco moyenné
            cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, 
                            rot_vec_aruco, mid_cam.reshape(1, 3), axis_length)
            
            return img, mid_base, self.aruco_axes, valid_markers
        
        return img, None, None, None
    
    def visualize_target_point(self, img, axis='z', distance=0.1):
        """
        Visualiser le point cible selon un des axes des ArUco
        axis: 'x', 'y', ou 'z' pour choisir l'axe de déplacement
        distance: distance à parcourir en mètres
        """
        if self.mid_base is None or self.aruco_axes is None:
            cv2.putText(img, "Aucun point cible détecté", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return img, None
        
        # Obtenir la direction selon l'axe choisi
        axis_key = axis.lower()
        if axis_key not in self.aruco_axes:
            print(f"Axe '{axis}' invalide, utilisation de Z")
            axis_key = 'z'
        
        direction_vector = self.aruco_axes[axis_key]
        
        # Appliquer la correction d'échelle
        corrected_distance = distance * self.scale_correction
        
        # Calculer le point cible selon la direction et la distance corrigée
        target_point = self.mid_base + corrected_distance * direction_vector
        
        # Affichage du point cible (afficher la distance demandée, pas la corrigée)
        txt = f"Point cible ({axis}, {distance}m): [{target_point[0]:.3f}, {target_point[1]:.3f}, {target_point[2]:.3f}]"
        cv2.putText(img, txt, (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Projection du point cible dans l'image pour visualisation
        # Transformer du repère base robot vers caméra
        hom = np.ones(4)
        hom[:3] = target_point
        target_cam = np.linalg.inv(self.T_cam_to_base) @ hom
        target_cam = target_cam[:3]
        
        try:
            # Vérification si le point est devant la caméra (z > 0)
            if target_cam[2] > 0:
                proj = rs.rs2_project_point_to_pixel(self.depth_intrin, target_cam.tolist())
                u, v = int(proj[0]), int(proj[1])
                if 0 <= u < 640 and 0 <= v < 480:  # Vérifier que c'est dans l'image
                    cv2.circle(img, (u, v), 8, (0, 0, 255), -1)
                    cv2.line(img, (int(proj[0]), int(proj[1])), 
                           (int(self.last_midpoint_px[0]), int(self.last_midpoint_px[1])), 
                           (0, 0, 255), 2)
                    cv2.putText(img, f"Cible ({axis})", (u + 10, v),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        except Exception as e:
            print(f"Erreur lors de la projection du point cible: {e}")
        
        return img, target_point

    def move_to_target(self, axis='z', distance=0.1, speed=0.1):
        """
        Déplacer le robot UR5 selon un des axes des ArUco
        axis: 'x', 'y', ou 'z' pour choisir l'axe de déplacement
        distance: distance à parcourir en mètres
        speed: vitesse du mouvement en m/s
        """
        if self.mid_base is None or self.aruco_axes is None:
            print("Aucune pose ArUco détectée. Impossible de déplacer le robot.")
            return False
        
        # Obtenir la direction selon l'axe choisi
        axis_key = axis.lower()
        if axis_key not in self.aruco_axes:
            print(f"Axe '{axis}' invalide, utilisation de Z")
            axis_key = 'z'
        
        direction_vector = self.aruco_axes[axis_key]
        
        # Appliquer la correction d'échelle
        corrected_distance = distance * self.scale_correction
        
        # Calculer le point cible avec la distance corrigée
        target_point = self.mid_base + corrected_distance * direction_vector
        
        if not UR5_AVAILABLE or self.ur5_control is None:
            print("Robot UR5 non disponible ou non connecté. Mode simulation uniquement.")
            print(f"Simulation: Déplacement selon axe {axis} sur {distance}m (corrigé: {corrected_distance:.2f}m) vers {target_point}")
            return False
        
        try:
            # Obtenir la pose actuelle du robot
            current_pose = self.ur5_receive.getActualTCPPose()
            
            # Préparer la nouvelle pose (position + orientation)
            new_pose = current_pose.copy()
            # Modifier seulement la position XYZ, garder la même orientation
            new_pose[0:3] = target_point
            
            # Déplacer le robot à la position cible avec la vitesse spécifiée
            self.ur5_control.moveL(new_pose, speed, 0.1)
            print(f"Robot déplacé vers: {target_point}")
            return True
        
        except Exception as e:
            print(f"Erreur lors du déplacement du robot: {e}")
            return False

    def run(self):
        """Boucle principale de l'application"""
        cv2.namedWindow("Détection ArUco", cv2.WINDOW_NORMAL)
        print("Appuyez sur:")
        print("  'q' pour quitter")
        print("  'v' pour visualiser le point cible")
        print("  'x', 'y', 'z' pour changer l'axe de déplacement")
        print("  '+', '-' pour modifier la distance de déplacement")
        print("  'm' pour déplacer le robot vers le point cible")
        print(f"  Facteur de correction d'échelle appliqué: {self.scale_correction:.2f}")
        
        show_target = False
        current_axis = 'y'  # Axe par défaut
        target_distance = 0.05  # Distance par défaut en mètres
        self.last_midpoint_px = (320, 240)  # Point milieu par défaut au centre de l'image
        
        while True:
            # 1) Lecture et alignement des flux
            frames = self.pipeline.wait_for_frames(10000)
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            
            img = np.asanyarray(color_frame.get_data())
            
            # 2) Détection des ArUco et calcul de la pose
            img, mid_point, axes, selected_ids = self.detect_aruco_with_pose(img, depth_frame)
            
            if mid_point is not None:
                self.mid_base = mid_point
                
                # Mettre à jour la position du point milieu dans l'image pour le dessin des lignes
                # Transformer le point milieu de base robot à caméra
                hom = np.ones(4)
                hom[:3] = mid_point
                mid_cam = np.linalg.inv(self.T_cam_to_base) @ hom
                mid_cam = mid_cam[:3]
                
                # Projeter dans l'image
                try:
                    if mid_cam[2] > 0:  # Devant la caméra
                        proj = rs.rs2_project_point_to_pixel(self.depth_intrin, mid_cam.tolist())
                        self.last_midpoint_px = proj
                except Exception:
                    pass  # Garder l'ancienne valeur en cas d'erreur
            
            # Afficher informations sur les paramètres actuels
            cv2.putText(img, f"Axe: {current_axis.upper()}, Distance: {target_distance:.2f}m (corrigée: {target_distance * self.scale_correction:.2f}m)", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 2)
            
            # 3) Visualisation du point cible si demandé
            if show_target and self.mid_base is not None and axes is not None:
                img, _ = self.visualize_target_point(img, axis=current_axis, distance=target_distance)
            
            # 4) Affichage des axes détectés
            if axes is not None:
                x_text = f"Axe X: [{axes['x'][0]:.2f}, {axes['x'][1]:.2f}, {axes['x'][2]:.2f}]"
                y_text = f"Axe Y: [{axes['y'][0]:.2f}, {axes['y'][1]:.2f}, {axes['y'][2]:.2f}]"
                z_text = f"Axe Z: [{axes['z'][0]:.2f}, {axes['z'][1]:.2f}, {axes['z'][2]:.2f}]"
                
                cv2.putText(img, x_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.putText(img, y_text, (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(img, z_text, (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
            # 5) Affichage final
            cv2.imshow("Détection ArUco", img)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('v'):
                show_target = not show_target
                print(f"Visualisation du point cible: {'activée' if show_target else 'désactivée'}")
            elif key == ord('x'):
                current_axis = 'x'
                print(f"Axe de déplacement: {current_axis}")
            elif key == ord('y'):
                current_axis = 'y'
                print(f"Axe de déplacement: {current_axis}")
            elif key == ord('z'):
                current_axis = 'z'
                print(f"Axe de déplacement: {current_axis}")
            elif key == ord('+') or key == ord('='):  # '=' est souvent le même touche que '+' sans shift
                target_distance += 0.05
                print(f"Distance: {target_distance:.2f}m (corrigée: {target_distance * self.scale_correction:.2f}m)")
            elif key == ord('-'):
                target_distance = max(0.05, target_distance - 0.05)
                print(f"Distance: {target_distance:.2f}m (corrigée: {target_distance * self.scale_correction:.2f}m)")
            elif key == ord('m'):
                # Déplacement du robot avec l'axe et la distance actuels
                if self.move_to_target(axis=current_axis, distance=target_distance):
                    print(f"Déplacement terminé selon axe {current_axis} sur {target_distance}m (corrigée: {target_distance * self.scale_correction:.2f}m)")
                else:
                    print("Déplacement impossible.")
        
        self.pipeline.stop()
        cv2.destroyAllWindows()
        
        # Fermer la connexion au robot si elle est ouverte
        if self.ur5_control:
            self.ur5_control.disconnect()

# --- Point d'entrée principal ---
if __name__ == "__main__":
    tracker = ArucoTracker()
    tracker.run()
