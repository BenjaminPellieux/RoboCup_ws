cmake_minimum_required(VERSION 3.16)
project(robocup_2027
  VERSION 0.0.0
  DESCRIPTION "RoboCup 2027 - Robot Autonome et Téléopéré"
  LANGUAGES CXX
)

## Compile as C++17 for ROS 2
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  matplotlib-cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Traitement_HSV.srv"
  "srv/Pas.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

###################################
## ament specific configuration  ##
###################################

ament_package()

###########
## Build ##
###########


## Declare a C++ executable

# Ajouter les exécutables
add_executable(${PROJECT_NAME}_QRcode_node src/QRCode/QRcode.cpp)
add_executable(${PROJECT_NAME}_republish_joy_node src/Deplacement_Robot/republish_joy.cpp)
add_executable(${PROJECT_NAME}_lot_camera_node src/Camera_Economique/Rescue.cpp)
add_executable(${PROJECT_NAME}_node src/Camera_Economique/Rescue.cpp)
add_executable(${PROJECT_NAME}_copie_trajet_node src/Lidar/Copie_Trajet.cpp)
add_executable(${PROJECT_NAME}_Exploration_automatique_intelligente_node src/Lidar/Exploration_automatique_intelligente.cpp)
add_executable(${PROJECT_NAME}_Initialisation_Accelero_node src/Lidar/Initialisation_Accelero.cpp)
add_executable(${PROJECT_NAME}_Bras_Automatique_node src/Bras_Automatique/Bras_Automatique.cpp src/Bras_Automatique/Finder.cpp)
add_executable(${PROJECT_NAME}_HSV_node src/Bras_Automatique/HSV_CAT.cpp)
add_executable(${PROJECT_NAME}_recuperation_data_auto_node src/Lidar/recuperation_data_auto.cpp)
add_executable(${PROJECT_NAME}_Exploration_Publisher_node src/Lidar/Exploration_Publisher.cpp)

# Lier les bibliothèques avec les exécutables
target_link_libraries(${PROJECT_NAME}_QRcode_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_republish_joy_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lot_camera_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_copie_trajet_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_Exploration_automatique_intelligente_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_Exploration_Publisher_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_Initialisation_Accelero_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_Bras_Automatique_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_HSV_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_recuperation_data_auto_node ${OpenCV_LIBRARIES})

# Ajouter les dépendances ROS 2
ament_target_dependencies(${PROJECT_NAME}_QRcode_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_republish_joy_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_lot_camera_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_copie_trajet_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_Exploration_automatique_intelligente_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_Exploration_Publisher_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_Initialisation_Accelero_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_Bras_Automatique_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_HSV_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)
ament_target_dependencies(${PROJECT_NAME}_recuperation_data_auto_node rclcpp std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros tf2)

# Installer les exécutables
install(TARGETS
  ${PROJECT_NAME}_QRcode_node
  ${PROJECT_NAME}_republish_joy_node
  ${PROJECT_NAME}_lot_camera_node
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_copie_trajet_node
  ${PROJECT_NAME}_Exploration_automatique_intelligente_node
  ${PROJECT_NAME}_Initialisation_Accelero_node
  ${PROJECT_NAME}_Bras_Automatique_node
  ${PROJECT_NAME}_HSV_node
  ${PROJECT_NAME}_recuperation_data_auto_node
  ${PROJECT_NAME}_Exploration_Publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installer les headers
install(DIRECTORY include/
  DESTINATION include
)






















