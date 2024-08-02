cmake_minimum_required(VERSION 3.5)
project(simple_amr_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds rviz
    DESTINATION share/${PROJECT_NAME}/
)


ament_package()
