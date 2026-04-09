#!/bin/bash

# Fonction pour exécuter une commande via SSH et sourcer l'environnement ROS
execute_ssh_command() {
    local host=$1
    local command=$2
    ssh ros@$host "bash -c 'source /opt/ros/noetic/setup.bash && $command'"
}

# Lancer roscore à distance sur la machine distante 192.168.137.80
execute_ssh_command "192.168.137.80" "roscore" &

# Attendre que roscore soit opérationnel (vous pouvez ajuster le délai si nécessaire)
sleep 10

# S'assurer que l'environnement ROS local est chargé
source /opt/ros/noetic/setup.bash

# Définir les paramètres ROS localement sur 192.168.137.82
rosparam set joy_node/dev "/dev/input/js2"

# Attendre un moment avant de lancer les nœuds suivants
sleep 5

# Lancer joy_node localement
rosrun joy joy_node &

# Attendre un moment avant de lancer le nœud suivant
sleep 5

# Lancer robocup2024_republish_joy_node localement
rosrun robocup2024 robocup2024_republish_joy_node &

# Attendre avant de lancer le nœud suivant (ajustez le délai si nécessaire)
sleep 10

# Exécuter rosserial_python serial_node.py à distance sur 192.168.137.80
execute_ssh_command "192.168.137.80" "rosrun rosserial_python serial_node.py /dev/ttyACM0" &
