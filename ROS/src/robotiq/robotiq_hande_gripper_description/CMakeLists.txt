cmake_minimum_required(VERSION 3.5)
project(robotiq_hande_gripper_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#ament_export_dependencies(xacro)
#ament_export_dependencies(urdf)
#ament_package()
