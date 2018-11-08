cmake_minimum_required(VERSION 3.5)
project(rqt_gui_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(qt_gui REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${qt_gui_INCLUDE_DIRS}
  ${qt_gui_cpp_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

SET(rqt_gui_cpp_SRCS
  # src/rqt_gui_cpp/component_plugin_provider.cpp
  # src/rqt_gui_cpp/roscpp_plugin_provider.cpp
)

set(rqt_gui_cpp_HDRS
  src/rqt_gui_cpp/roscpp_plugin_provider.h
)

qt5_wrap_cpp(rqt_gui_cpp_MOCS ${rqt_gui_cpp_HDRS})

add_library(${PROJECT_NAME}
  ${rqt_gui_cpp_SRCS}
  ${rqt_gui_cpp_MOCS})

target_link_libraries(
  ${PROJECT_NAME}
  ${qt_gui_cpp_LIBRARIES}
  Qt5::Widgets
)

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE "URDF_BUILDING_DLL")
endif()

if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(Qt5Widgets)
ament_export_libraries(${PROJECT_NAME})

ament_package()
