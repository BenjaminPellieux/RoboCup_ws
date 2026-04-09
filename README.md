# RoboCup 2024
## Projet Robotique Autonome et Téléopéré

Ce projet ROS est développé pour la compétition RoboCup 2024 aux Pays-Bas. Il intègre plusieurs modules pour la vision par ordinateur, la navigation autonome, la manipulation d'objets et l'interface homme-machine.

---

## 📋 Table des matières

- [Architecture Générale](#architecture-générale)
- [Prérequis](#prérequis)
- [Installation et Compilation](#installation-et-compilation)
- [Lancement du Projet](#lancement-du-projet)
- [Modules Disponibles](#modules-disponibles)
- [Services ROS](#services-ros)
- [Topics ROS](#topics-ros)
- [Interface Utilisateur](#interface-utilisateur)
- [Configuration](#configuration)
- [Dépannage](#dépannage)

---

## Architecture Générale

Le projet est organisé en plusieurs modules spécialisés :

```
RoboCup_ws/
├── src/
│   ├── Bras_Automatique/        # Contrôle automatisé du bras
│   ├── Bras_Manuel/             # Contrôle manuel du bras
│   ├── Camera_Economique/       # Module caméra bas coût
│   ├── Deplacement_Robot/       # Navigation et déplacement
│   ├── Detection_Mouvement/     # Détection de mouvement
│   ├── IHM_Robot/               # Interface Web de contrôle
│   ├── Lidar/                   # Système Lidar
│   ├── Pince_Bras/              # Contrôle de la pince
│   ├── QRCode/                  # Détection des codes QR
│   └── Accelerometre/           # Capteur accéléromètre (IMU)
├── launch/                       # Fichiers de lancement ROS
├── msg/                          # Définitions de messages ROS
├── srv/                          # Définitions de services ROS
├── matplotlib-cpp/               # Bibliothèque de visualisation
└── scripts/                      # Scripts utilitaires
```

---

## Prérequis

### Système d'exploitation
- Ubuntu 18.04 ou 20.04 (recommandé pour ROS Noetic)
- Ubuntu 16.04 (compatible avec ROS Kinetic)

### Dépendances Système
```bash
# Mise à jour des dépendances système
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  libopencv-dev \
  python3-dev \
  libpcl-dev \
  python3-pip
```

### ROS et Outils
- **ROS Noetic** ou **Kinetic** (voir installation officielle : http://wiki.ros.org/ROS/Installation)
- **catkin** (gestionnaire de build pour ROS)
- **cv_camera** (capteur caméra OpenCV)
- **rosserial** (communication série)
- **joy** (support manette de jeu)

### Dépendances C++
- OpenCV 3.0+
- PCL (Point Cloud Library)
- Python 3.x
- matplotlib-cpp (inclus)

---

## Installation et Compilation

### 1. Cloner le workspace
```bash
cd ~/
git clone https://github.com/serrhiniabdellah/RoboCup_ws ros_workspaces/RoboCup_ws
cd ros_workspaces/RoboCup_ws
```

### 2. Installer les dépendances ROS
```bash
# Installer les dépendances manquantes via rosdep
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Sourcer l'environnement ROS
```bash
source /opt/ros/noetic/setup.bash  # Pour ROS Noetic
# OU
source /opt/ros/kinetic/setup.bash  # Pour ROS Kinetic

# Ajouter à votre ~/.bashrc pour source automatique
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Compiler avec catkin

```bash
cd ~/ros_workspaces/RoboCup_ws
catkin_make

# Ou pour un nettoyage complet avant recompilation :
catkin_make clean
catkin_make
```

### 5. Sourcer votre workspace après compilation
```bash
source ~/ros_workspaces/RoboCup_ws/devel/setup.bash

# Ajouter au ~/.bashrc pour source automatique
echo "source ~/ros_workspaces/RoboCup_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Lancement du Projet

### Lancement Rapide - Configuration Complète
```bash
# Option 1 : Bras Automatique avec Caméra
roslaunch Robocup2024_PaysBas Bras.launch

# Option 2 : Caméra Haute Résolution
roslaunch Robocup2024_PaysBas Flux_8MP.launch

# Option 3 : Système Lidar
roslaunch Robocup2024_PaysBas Lidar.launch

# Option 4 : Initialisation Accéléromètre
roslaunch Robocup2024_PaysBas Init_Accelero.launch

# Option 5 : Mode Sauvetage (Rescue)
roslaunch Robocup2024_PaysBas Rescue.launch
```

### Lancement Étape par Étape
```bash
# Terminal 1 : Démarrer roscore
roscore

# Terminal 2 : Lancer le nœud caméra
roslaunch Robocup2024_PaysBas Bras.launch

# Terminal 3 : Visualiser les images (optionnel)
rosrun rqt_image_view rqt_image_view

# Terminal 4 : Visualiser les données (optionnel)
rosrun rviz rviz
```

### Lancement Automatisé Complet
```bash
# Lancer tous les modules avec configuration automatique
bash scripts/launch_all.sh
```

---

## Modules Disponibles

### 🦾 Bras Automatique
**Fichiers** : `src/Bras_Automatique/`

Permet la détection et manipulation automatique d'objets :
- Détection d'objets via HSV (teinte, saturation, valeur)
- Calcul de position en 3D
- Contrôle automatisé du bras
- Calibration HSV configurable

**Utilisation** :
```bash
roslaunch Robocup2024_PaysBas Bras.launch
```

### 🎮 Bras Manuel
**Fichiers** : `src/Bras_Manuel/`

Contrôle du bras via manette de jeu ou clavier :
- Interface ROS
- Configuration des plages moteurs
- URDF pour simulation

**Lancement** : Utiliser la manette via le module Deplacement_Robot

### 📷 Caméra Économique
**Fichiers** : `src/Camera_Economique/`

Système de vision bas coût :
- Capture vidéo en temps réel
- Support caméra USB standard
- Optimisé pour faible latence

**Paramètres configurables** :
- Résolution : 320×240 ou 1920×1440
- Périphérique : `/dev/video0`, `/dev/video1`, etc.
- Fréquence d'images

### 🚀 Déplacement Robot
**Fichiers** : `src/Deplacement_Robot/`

Gestion de la locomotion du robot :
- Classe `sabertooth` pour contrôle moteur
- Support manette de jeu
- Republication des commandes de joystick

**Topics** :
- `/joy` : Données brutes manette
- `/cmd_vel` : Commandes de vitesse

### 👁 Détection de Mouvement
**Fichiers** : `src/Detection_Mouvement/`

Analyse vidéo pour détection de mouvement :
- Scripts Python pour flux vidéo continu
- Configuration en temps réel
- Seuils ajustables

**Scripts** :
- `Detection_Mouvement.py` : Mode unique
- `Detection_Mouvement_Continu.py` : Flux continu

### 📊 Lidar
**Fichiers** : `src/Lidar/`

Système de cartographie 3D et navigation :
- Traitement des nuages de points
- Exploration autonome
- Accéléromètre intégré

### 🔤 QR Code
**Fichiers** : `src/QRCode/`

Détection et décodage de codes QR :
- Localisation précise
- Extraction de données

### 📈 Accéléromètre (IMU)
**Fichiers** : `src/Accelerometre/`

Capteur inertiel (MPU-9250) :
- Mesure accélération 3D
- Orientation gyroscope
- Calibration disponible

---

## Services ROS

### Service : Traitement_HSV
Permet d'ajuster les paramètres HSV pour la détection d'objets

**Fichier** : `srv/Traitement_HSV.srv`

**Paramètres** :
```
int64 LowH        # Teinte minimale (0-180)
int64 HighH       # Teinte maximale (0-180)
int64 LowS        # Saturation minimale (0-255)
int64 HighS       # Saturation maximale (0-255)
int64 LowV        # Valeur minimale (0-255)
int64 HighV       # Valeur maximale (0-255)
```

**Utilisation** :
```bash
# Ajuster la détection HSV pendant l'exécution
rosservice call /Traitement_HSV 30 180 100 255 100 255

# Pour faciliter la calibration, utiliser :
opencv3_V2_node
```

### Service : Pas
Contrôle du déplacement pas à pas

**Fichier** : `srv/Pas.srv`

---

## Topics ROS

### Messages Publiés

#### `pieces_info` - Information sur les objets détectés
```plaintext
Header header       # Timestamp et frame_id
int32 rang         # Rang/ID de l'objet
float32 x1, y1     # Coin supérieur gauche
float32 x2, y2     # Coin inférieur droit
float32 center_x   # Centre X
float32 center_y   # Centre Y
float32 width      # Largeur
float32 height     # Hauteur
```

#### `nb_pieces` - Nombre d'objets détectés
```plaintext
int32 count        # Nombre total d'objets
```

#### `donnees_comparaison` - Données de comparaison
Utilisé pour l'analyse des performances des détections

### Topics Standards ROS

- `/camera/image_raw` : Flux vidéo caméra
- `/camera/camera_info` : Calibration caméra
- `/joy` : Données brutes manette
- `/cmd_vel` : Commandes de vitesse
- `/tf` : Transformations géométriques
- `/scan` : Données Lidar

---

## Interface Utilisateur

### Web IHM
**Fichiers** : `src/IHM_Robot/`

Interface HTML/CSS/JavaScript pour le contrôle du robot :

```
index.html                  # Page d'accueil principale
├── deplacement_robot.html  # Contrôle du déplacement
├── deplacement_robot.js    # Logique déplacement
├── vision_camera.html      # Flux caméra
├── vision_camera.js        # Logique caméra
├── vision_lidar.html       # Visualisation Lidar
├── vision_lidar.js         # Logique Lidar
├── deplacement_bras.html   # Contrôle du bras
├── deplacement_bras.js     # Logique du bras
└── style.css               # Styles
```

### Accès à l'IHM
```bash
# Lancer un serveur web local
cd src/IHM_Robot/
python3 -m http.server 8000

# Accéder via navigateur
# http://localhost:8000
# ou http://<IP_ROBOT>:8000
```

---

## Configuration

### Fichier de Configuration HSV
**Fichier** : `HSV_Config.txt`

Contient les paramètres HSV prédéfinis pour différents objets :
```
# Format : NomObjet LowH HighH LowS HighS LowV HighV
Red 0 10 100 255 100 255
Green 40 80 100 255 100 255
Blue 100 130 100 255 100 255
```

### Configuration Caméra
**Dans les fichiers .launch** :
```xml
<param name="device_path" value="/dev/video0" />
<param name="image_width" value="320" />
<param name="image_height" value="240" />
```

### Configuration Moteurs (Bras Manuel)
**Fichier** : `src/Bras_Manuel/Ranges_moteurs`

Plages de mouvement pour chaque articulation du bras

---

## Dépannage

### Erreur de Compilation
```bash
# Nettoyer et recompiler
catkin_make clean
rm -rf build devel
catkin_make

# Ou utiliser ccache pour plus rapide
catkin_make -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
```

### Caméra non détectée
```bash
# Vérifier les périphériques vidéo disponibles
ls -la /dev/video*

# Tester la caméra
v4l2-ctl --list-devices
```

### Problèmes de Communication Série
```bash
# Vérifier le port USB
ls -la /dev/ttyACM*
ls -la /dev/ttyUSB*

# Affecter les permissions
sudo usermod -a -G dialout $USER
# Redémarrer la session ou :
newgrp dialout
```

### Problèmes ROS
```bash
# Vérifier la connectivité ROS
rosnode list
rostopic list

# Déboguer les nœuds
rostopic echo /topic_name
rosservice list
```

### Manque de dépendances
```bash
# Installer les dépendances manquantes
rosdep install --from-paths src --ignore-src -r -y

# Installer des packages Python spécifiques
pip3 install numpy opencv-python
```

### Performance faible
```bash
# Vérifier les ressources
htop
# Regarder utilisation CPU/RAM

# Réduire la résolution caméra
# Modifier dans les fichiers .launch
```

---

## Documentation Additionnelle

- **ROS Documentation** : http://wiki.ros.org/
- **OpenCV Documentation** : https://docs.opencv.org/
- **PCL Documentation** : https://pointclouds.org/documentation/
- **RoboCup Official** : https://www.robocup.org/

---

## Notes Importantes

- ⚠️ Assurez-vous que `roscore` est lancé avant les autres nœuds
- ⚠️ Vérifiez les permissions USB pour les périphériques série
- ℹ️ Les paramètres HSV doivent être calibrés selon l'éclairage ambiant
- ℹ️ Utilisez `rqt` pour monitorer les topics et debugger en temps réel

---

## Licence

À définir

## Mainteneurs

- Dominik (dominik@todo.todo)

---

**Dernière mise à jour** : Mars 2026
    
