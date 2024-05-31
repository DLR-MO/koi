cmake_minimum_required(VERSION 3.5)
project(koi)

find_package(ament_cmake REQUIRED)

ament_python_install_package(koi)
install(DIRECTORY scripts USE_SOURCE_PERMISSIONS DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY demo/scripts USE_SOURCE_PERMISSIONS DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY demo/config DESTINATION share/${PROJECT_NAME}/demo)
install(DIRECTORY demo/models DESTINATION share/${PROJECT_NAME}/demo)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
