import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Initialisation de ROS
rospy.init_node('motion_detector', anonymous=True)

# Initialisation du bridge pour convertir les images ROS en images OpenCV
bridge = CvBridge()

# Variables globales pour les images
frame1 = None
frame2 = None

def image_callback(msg):
	global frame1, frame2

	try:
		# Conversion du message ROS en image OpenCV
		frame = bridge.imgmsg_to_cv2(msg, "bgr8")
		
		# Initialisation des frames
		if frame1 is None:
			frame1 = frame
			return
		
		if frame2 is None:
			frame2 = frame
			return
		
		# Calcul de la différence absolue entre deux images consécutives
		diff = cv2.absdiff(frame1, frame2)
		
		# Conversion en niveau de gris
		gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
		
		# Application d'un flou pour réduire le bruit
		blur = cv2.GaussianBlur(gray, (5, 5), 0)
		
		# Seuil pour obtenir une image binaire
		_, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
		
		# Dilatation de l'image pour combler les trous
		dilated = cv2.dilate(thresh, None, iterations=3)
		
		# Recherche des contours
		contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		
		for contour in contours:
			# Filtrage des petits mouvements
			if cv2.contourArea(contour) < 500:
				continue
			
			# Dessin des rectangles autour des mouvements détectés
			(x, y, w, h) = cv2.boundingRect(contour)
			cv2.rectangle(frame1, (x, y), (x + w, y + h), (0, 255, 0), 2)
		
		# Affichage de l'image
		cv2.imshow("feed", frame1)
		
		# Mise à jour des images
		frame1 = frame2
		frame2 = frame
		
		# Sortie de la boucle sur appui de la touche 'q'
		if cv2.waitKey(40) == ord('q'):
			rospy.signal_shutdown('Quit')
		
	except CvBridgeError as e:
		print(e)

# Abonnement au topic de la caméra
image_sub = rospy.Subscriber('/cv_camera/image_raw', Image, image_callback)

# Boucle principale ROS
rospy.spin()

# Libération des ressources
cv2.destroyAllWindows()
