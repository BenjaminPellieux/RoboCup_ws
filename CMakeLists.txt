cmake_minimum_required(VERSION 2.8.3)
project(Robocup2024_PaysBas)

## Compile as C++11, supported in ROS Kinetic and newer 
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
  geometry_msgs
  tf2_ros
  tf2
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(PythonLibs 3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  /home/ros/catkin_ws/src/Robocup2024_PaysBas/matplotlib-cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Traitement_HSV.srv
  Pas.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime cv_bridge sensor_msgs
  DEPENDS PCL OpenCV
)

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
target_link_libraries(${PROJECT_NAME}_QRcode_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_republish_joy_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lot_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_copie_trajet_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_Exploration_automatique_intelligente_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_Exploration_Publisher_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_Initialisation_Accelero_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_Bras_Automatique_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_HSV_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_recuperation_data_auto_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_Exploration_Publisher_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Ajouter des dépendances pour les messages générés
add_dependencies(${PROJECT_NAME}_republish_joy_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_lot_camera_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_Bras_Automatique_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_HSV_node ${PROJECT_NAME}_generate_messages_cpp)























