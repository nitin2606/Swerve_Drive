cmake_minimum_required(VERSION 3.8)
project(swerve_drive_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds)

install(DIRECTORY models/
        DESTINATION share/${PROJECT_NAME}/models)

install(DIRECTORY urdf/
        DESTINATION share/${PROJECT_NAME}/urdf)



install(DIRECTORY meshes/
        DESTINATION share/${PROJECT_NAME}/meshes)


install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
