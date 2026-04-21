# RoboCup 2024
## Projet Robotique Autonome et Téléopéré

Ce projet ROS est développé pour la compétition RoboCup 2027 aux Pays-Bas. Il intègre plusieurs modules pour la vision par ordinateur, la navigation autonome, la manipulation d'objets et l'interface homme-machine.

---

## Table des matières

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
- Ubuntu 24.04 (Noble)

### Installation de ROS 2 Jazzy
Suivez la documentation officielle : https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```bash
# 1. Configurer les paramètres régionaux (locale)
locale  # Vérifier la support UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2. Activer le dépôt Universe
sudo apt install software-properties-common
sudo add-apt-repository universe

# 3. Ajouter les sources ROS 2
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# 4. Installer ROS 2 Jazzy
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y

# 5. Installer les outils de développement
sudo apt install ros-dev-tools -y
```

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
  python3-pip \
  python3-colcon-common-extensions
```

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
git clone https://github.com/BenjaminPellieux/RoboCup_ws ros_workspaces/RoboCup_ws
cd ros_workspaces/RoboCup_ws
```

### 2. Installer les dépendances ROS 2
```bash
# Installer les dépendances manquantes via rosdep
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Sourcer l'environnement ROS 2 Jazzy

Avant de compiler et d'utiliser le workspace, toujours sourcer d'abord ROS 2 :

```bash
source /opt/ros/jazzy/setup.bash
```

**Pour sourcer automatiquement à chaque ouverture de terminal** :

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Compiler avec colcon

```bash
cd ~/ros_workspaces/RoboCup_ws
colcon build

# Ou pour un nettoyage complet avant recompilation :
colcon build --symlink-install --cmake-clean-first
```

### 5. Sourcer votre workspace après compilation

Après la compilation, un dossier `install/` est créé avec automatiquement un fichier `setup.bash` :

```bash
source ~/ros_workspaces/RoboCup_ws/install/setup.bash

# Ajouter au ~/.bashrc pour source automatique
echo "source ~/ros_workspaces/RoboCup_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Important** : Toujours sourcer ROS 2 d'abord, puis votre workspace :

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros_workspaces/RoboCup_ws/install/setup.bash
```

---

## Explication du fichier setup.bash

Il y a **deux fichiers `setup.bash` distincts** :

### 1. Setup du système ROS 2
**Chemin** : `/opt/ros/jazzy/setup.bash`

- Créé automatiquement lors de l'installation de ROS 2 via apt
- À sourcer en **premier**
- Configure les variables d'environnement ROS de base
- Disponible pour tous les projets

### 2. Setup du workspace
**Chemin** : `~/ros_workspaces/RoboCup_ws/install/setup.bash`

- Créé **automatiquement** par `colcon build`
- À sourcer en **second**
- Configure l'accès aux packages et dépendances du workspace
- Vous n'avez pas besoin de le créer manuellement

**Ordre correct d'initialisation** :
```bash
# D'abord : le système ROS 2
source /opt/ros/jazzy/setup.bash

# Ensuite : votre workspace
source ~/ros_workspaces/RoboCup_ws/install/setup.bash

# Maintenant vous pouvez utiliser ros2 et lancer vos nœuds
ros2 launch RoboCup_ws Bras.launch.py
```

---

## Lancement du Projet

### Lancement Rapide - Configuration Complète

Tous les modules cités ici ne sont pas fonctionnels sur cette version du RMS.

```bash
# Assurez-vous que ROS 2 et votre workspace sont sourcés
source /opt/ros/jazzy/setup.bash
source ~/ros_workspaces/RoboCup_ws/install/setup.bash

# Option 1 : Bras Automatique avec Caméra
ros2 launch RoboCup_ws Bras.launch.py

# Option 2 : Caméra Haute Résolution
ros2 launch RoboCup_ws Flux_8MP.launch.py

# Option 3 : Système Lidar
ros2 launch RoboCup_ws Lidar.launch.py

# Option 4 : Initialisation Accéléromètre
ros2 launch RoboCup_ws Init_Accelero.launch.py

# Option 5 : Mode Sauvetage (Rescue)
ros2 launch RoboCup_ws Rescue.launch.py
```

### Lancement Étape par Étape
```bash
# Terminal 1 : Lancer le nœud caméra
# (En ROS 2, pas besoin de roscore)
ros2 launch RoboCup_ws Bras.launch.py

# Terminal 2 : Visualiser les images (optionnel)
ros2 run rqt_image_view rqt_image_view

# Terminal 3 : Visualiser les données (optionnel)
ros2 run rviz2 rviz2
```

### Lancement Automatisé Complet
```bash
# Lancer tous les modules avec configuration automatique
bash scripts/launch_all.sh
```

---

## Modules Disponibles

### Caméra Économique
**Fichiers** : `src/Camera_Economique/`

Système de vision bas coût :
- Capture vidéo en temps réel
- Support caméra USB standard
- Optimisé pour faible latence

**Paramètres configurables** :
- Résolution : 320×240 ou 1920×1440
- Périphérique : `/dev/video0`, `/dev/video1`, etc.
- Fréquence d'images

### Déplacement Robot
**Fichiers** : `src/Deplacement_Robot/`

Gestion de la locomotion du robot :
- Classe `sabertooth` pour contrôle moteur
- Support manette de jeu
- Republication des commandes de joystick

**Topics** :
- `/joy` : Données brutes manette
- `/cmd_vel` : Commandes de vitesse

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
ros2 service call /Traitement_HSV robocupws_interfaces/srv/TraitementHSV "{lowh: 30, highh: 180, lows: 100, highs: 255, lowv: 100, highv: 255}"

# Pour faciliter la calibration, utiliser :
# opencv3_V2_node
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
colcon build --cmake-clean-first

# Ou un nettoyage complet
rm -rf build install log
colcon build
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

### Problèmes ROS 2
```bash
# Vérifier les nœuds en cours d'exécution
ros2 node list

# Lister les topics disponibles
ros2 topic list

# Écouter un topic spécifique
ros2 topic echo /topic_name

# Lister les services
ros2 service list
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
# Modifier dans les fichiers .launch.py
```

### Sourcing incorrect
Si vous rencontrez des erreurs comme "command not found: ros2" :

```bash
# Vérifier que ROS 2 est sourcé
source /opt/ros/jazzy/setup.bash

# Vérifier que votre workspace est sourcé
source ~/ros_workspaces/RoboCup_ws/install/setup.bash

# Vérifier l'ordre d'exécution (ROS 2, puis workspace)
echo $ROS_DISTRO  # Doit afficher 'jazzy'
```

---

## Documentation Additionnelle

- **ROS Documentation** : http://wiki.ros.org/
- **OpenCV Documentation** : https://docs.opencv.org/
- **PCL Documentation** : https://pointclouds.org/documentation/
- **RoboCup Official** : https://www.robocup.org/

---

## Notes Importantes

- **ROS 2 Jazzy** n'utilise pas `roscore` : il est lancé automatiquement quand vous exécutez les nœuds
- Toujours sourcer ROS 2 d'abord : `source /opt/ros/jazzy/setup.bash`
- Puis sourcer votre workspace : `source ~/ros_workspaces/RoboCup_ws/install/setup.bash`
- Les fichiers de lancement sont maintenant en `.launch.py` (Python) au lieu de `.launch` (XML)
- Vérifiez les permissions USB pour les périphériques série
- Les paramètres HSV doivent être calibrés selon l'éclairage ambiant
- Utilisez `rqt` pour monitorer les topics et debugger en temps réel

---

**Dernière mise à jour** : Avril 2026