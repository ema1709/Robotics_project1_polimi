# Impostazioni minime per utilizzare Catkin
cmake_minimum_required(VERSION 2.8.3)
project(first_project)

# Aggiungi dipendenze per il pacchetto Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  message_generation
)
add_message_files(
FILES
custom.msg
)
generate_messages(
DEPENDENCIES
std_msgs
)

# Declara il pacchetto
catkin_package(
  
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf message_runtime
)

# Aggiungi le directory dove si trovano gli header
include_directories(
  ${catkin_INCLUDE_DIRS}
)


# Specifica dove si trovano i tuoi file sorgente
add_executable(gps_odometer src/gps_odometer.cpp)
add_executable(odometer src/odometer.cpp)
add_executable(sector_times src/sector_times.cpp)


add_dependencies(sector_times first_project_generate_messages_cpp)

# Collega le librerie richieste per ogni nodo
target_link_libraries(gps_odometer ${catkin_LIBRARIES})
target_link_libraries(odometer ${catkin_LIBRARIES})
target_link_libraries(sector_times ${catkin_LIBRARIES})





