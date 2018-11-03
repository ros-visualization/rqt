cmake_minimum_required(VERSION 3.5)
project(rqt_py_common)

# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${_library_name}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    test/msg/Val.msg
    test/msg/ArrayVal.msg
    DEPENDENCIES std_msgs)
  ament_add_pytest_test(${PROJECT_NAME} test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 90)
endif()

ament_package()