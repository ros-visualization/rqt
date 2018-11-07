cmake_minimum_required(VERSION 3.5)
project(rqt_gui)

# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

install(PROGRAMS bin/rqt_gui
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY resource
  DESTINATION share
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(${PROJECT_NAME} test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 90)
endif()

ament_package()
