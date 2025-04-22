import cv2
import numpy as np
import pyrealsense2 as rs

# --- Transformation fixe caméra → base robot (à calibrer) ---
T_cam_to_base = np.eye(4)
T_cam_to_base[:3,3] = [0.4, -0.2, 0.6]  # position de la caméra dans le repère robot

# --- Initialisation RealSense ---
pipeline = rs.pipeline()
cfg      = rs.config()
cfg.enable_stream(rs.stream.depth,  640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color,  640, 480, rs.format.bgr8,30)
profile  = pipeline.start(cfg)
align    = rs.align(rs.stream.color)
depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

# --- Utilitaire de déprojection pixel→3D caméra ---
def deproject(px, py, depth):
    return np.array(rs.rs2_deproject_pixel_to_point(depth_intrin, [px, py], depth), dtype=float)

# --- Paramètres ArUco ---
aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

cv2.namedWindow("Détection ArUco", cv2.WINDOW_NORMAL)
print("Appuie sur 'q' pour quitter.")

while True:
    # 1) Lecture et alignement des flux
    frames      = pipeline.wait_for_frames()
    aligned     = align.process(frames)
    depth_frame = aligned.get_depth_frame()
    color_frame = aligned.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    img = np.asanyarray(color_frame.get_data())

    # 2) Détection des ArUco
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = detector.detectMarkers(img)

    if ids is not None and len(ids) > 1:
        cv2.aruco.drawDetectedMarkers(img, corners, ids)

        # 3) Recherche de la première paire d'IDs consécutifs
        present = sorted(int(i[0]) for i in ids)
        pair = None
        for x in present:
            if x + 1 in present:
                pair = (x, x+1)
                break

        if pair:
            id1, id2 = pair
            pts3d = []
            # 4) Pour chaque marqueur de la paire, calcul de son point 3D caméra
            for corner, mid in zip(corners, ids):
                mid_id = int(mid[0])
                if mid_id in pair:
                    # centre pixel du marqueur
                    c = corner[0]
                    cx, cy = int(c[:,0].mean()), int(c[:,1].mean())

                    # patch 5×5 pour lisser la profondeur
                    depths = []
                    for dx in (-2,-1,0,1,2):
                        for dy in (-2,-1,0,1,2):
                            d = depth_frame.get_distance(cx+dx, cy+dy)
                            if d > 0:
                                depths.append(d)
                    if not depths:
                        continue
                    depth = float(np.median(depths))

                    # deprojection
                    p_cam = deproject(cx, cy, depth)
                    pts3d.append(p_cam)

                    # marquage visuel
                    cv2.circle(img, (cx, cy), 4, (255,0,0), -1)
                    cv2.putText(img, f"ID={mid_id}", (cx+5, cy-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

            # 5) Si on a bien deux points, calcul du milieu et affichage
            if len(pts3d) == 2:
                mid_cam = (pts3d[0] + pts3d[1]) / 2.0

                # transformation caméra→robot
                hom = np.ones(4); hom[:3] = mid_cam
                mid_base = (T_cam_to_base @ hom)[:3]

                # texte 3D
                txt = f"Milieu base({id1}-{id2}): [{mid_base[0]:.3f}, {mid_base[1]:.3f}, {mid_base[2]:.3f}]"
                cv2.putText(img, txt, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

                # projection dans l'image
                proj = rs.rs2_project_point_to_pixel(depth_intrin, mid_cam.tolist())
                u, v = int(proj[0]), int(proj[1])
                cv2.circle(img, (u, v), 6, (0,255,255), -1)

    # 6) Affichage final
    cv2.imshow("Détection ArUco", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()
