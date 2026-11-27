cmake_minimum_required(VERSION 3.20)
project(rqt_gui_cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)

if(WIN32)
  message(STATUS "rqt_gui_cpp is not yet supported on Windows. Package will not be built.")
  ament_package()
  return()
endif()

# The C++ standard (C++20) is provided by the
# ament_cmake_ros_core::ament_ros_defaults interface target linked below.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(pluginlib REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)

ament_export_dependencies(
  rclcpp
  qt_gui_cpp
)

add_library(${PROJECT_NAME} SHARED
  src/rqt_gui_cpp/nodelet_plugin_provider.cpp
  src/rqt_gui_cpp/plugin.cpp
  src/rqt_gui_cpp/roscpp_plugin_provider.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
  qt_gui_cpp::qt_gui_cpp
  rclcpp::rclcpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ament_cmake_ros_core::ament_ros_defaults
  pluginlib::pluginlib
  Qt6::Widgets
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plugin test/test_plugin.cpp)
  if(TARGET test_plugin)
    target_link_libraries(test_plugin
      ${PROJECT_NAME}
      rclcpp::rclcpp
      Qt6::Widgets
    )
  endif()

  add_library(test_plugin_ros SHARED ./test/test_plugin_ros.cpp)
  target_link_libraries(test_plugin_ros ${PROJECT_NAME})

  pluginlib_enable_plugin_testing(
    CMAKE_TARGET_VAR mock_install_target
    AMENT_PREFIX_PATH_VAR mock_install_path
    PACKAGE_NAME "test_rqt_gui_cpp"
    PACKAGE_XML "test/test_package.xml"
    PLUGIN_CATEGORY "rqt_gui"
    PLUGIN_DESCRIPTIONS "test/test_plugins.xml"
    PLUGIN_LIBRARIES test_plugin_ros
  )

  # Add the mock target to ALL because the following pytest call doesn't create
  # a target which could depend on the mock target.
  add_custom_target(${PROJECT_NAME}_plugin_testing_setup ALL
    DEPENDS "${mock_install_target}" test_plugin_ros
  )

  ament_add_pytest_test(${PROJECT_NAME}_test
    test/test_rqt_gui_cpp.py
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:test_plugin_ros>"
    APPEND_ENV AMENT_PREFIX_PATH=${mock_install_path}
  )
  set_tests_properties(${PROJECT_NAME}_test PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 120
  )
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(qt_gui "plugin.xml")

# Export old-style CMake variables
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_package()
