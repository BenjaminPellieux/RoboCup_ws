# RoboCup_ws

Bienvenue dans le dépôt du workspace ROS2 pour notre projet RoboCup.

## Table des matières

- [Présentation](#présentation)
- [Installation](#installation)
- [Utilisation](#utilisation)
- [Contribuer](#contribuer)
- [Code de conduite](#code-de-conduite)
- [Licence](#licence)

## Présentation

Ce projet est dédié au développement d'un robot compétitif pour la RoboCup, utilisant le framework ROS2.

## Installation

1. **Clonez le dépôt :**

   ```bash
   git clone https://github.com/BenjaminPellieux/RoboCup_ws.git
   ```

2. **Accédez au répertoire du workspace :**

   ```bash
   cd RoboCup_ws
   ```

3. **Installez les dépendances :**

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Compilez le workspace :**

   ```bash
   colcon build
   ```

## Utilisation

1. **Sourcez le workspace :**

   ```bash
   source install/setup.bash
   ```

2. **Lancez le robot :**

   ```bash
   ros2 launch <nom_du_package> <fichier_de_lancement.launch.py>
   ```

   *Remplacez `<nom_du_package>` et `<fichier_de_lancement.launch.py>` par les noms appropriés.*

## Contribuer

Nous accueillons les contributions ! Veuillez consulter le fichier [CONTRIBUTING.md](CONTRIBUTING.md) pour plus d'informations sur la façon de commencer.

## Code de conduite

Ce projet adhère au [Code de Conduite](CODE_OF_CONDUCT.md). En participant, vous vous engagez à respecter ses termes.

## Licence

Ce projet est sous licence MIT - voir le fichier [LICENSE](LICENSE) pour plus de détails.


