cmake_minimum_required(VERSION 3.5)
project(thymio_description)

find_package(urdf REQUIRED)
find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(image_transport REQUIRED)
#find_package(OpenCV REQUIRED)

#add_executable(image_to_luminance src/image_to_luminance.cpp)
#ament_target_dependencies(image_to_luminance rclcpp sensor_msgs)

#install(TARGETS
#  image_to_luminance
#  DESTINATION lib/${PROJECT_NAME})

install(FILES meshes/base_simple.dae meshes/base_collision.dae meshes/wheel.dae
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
