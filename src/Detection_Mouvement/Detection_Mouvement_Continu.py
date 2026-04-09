import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Initialisation de ROS
rospy.init_node('square_detector', anonymous=True)

# Initialisation du bridge pour convertir les images ROS en images OpenCV
bridge = CvBridge()

def detect_square(frame):
    # Conversion en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Application d'un flou pour réduire le bruit
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Utilisation du seuillage adaptatif pour obtenir une image binaire
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    
    # Recherche de contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Filtrage des petits contours
        if cv2.contourArea(contour) < 100:
            continue

        # Approximation du contour pour détecter les formes rectangulaires
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        if len(approx) == 4:
            # Détection de carré
            (x, y, w, h) = cv2.boundingRect(approx)
            aspect_ratio = w / float(h)
            if 0.95 <= aspect_ratio <= 1.05:  # Vérification de l'aspect ratio pour s'assurer que c'est un carré
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

def image_callback(msg):
    try:
        # Conversion du message ROS en image OpenCV
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Détection du carré noir
        detect_square(frame)
        
        # Affichage de l'image avec les carrés détectés
        cv2.imshow("feed", frame)
        
        # Sortie de la boucle sur appui de la touche 'q'
        if cv2.waitKey(1) == ord('q'):
            rospy.signal_shutdown('Quit')
        
    except CvBridgeError as e:
        print(e)

# Abonnement au topic de la caméra
image_sub = rospy.Subscriber('/cv_camera/image_raw', Image, image_callback)

# Boucle principale ROS
rospy.spin()

# Libération des ressources
cv2.destroyAllWindows()
