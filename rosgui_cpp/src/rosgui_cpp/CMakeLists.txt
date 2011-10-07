find_package(PythonLibs REQUIRED)
find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(rosgui_cpp_SRCS
  composite_plugin_provider.cpp
  generic_proxy.cpp
  main_window_interface.cpp
  plugin_bridge.cpp
  plugin_context.cpp
  plugin_descriptor.cpp
  plugin_provider.cpp
  recursive_plugin_provider.cpp
  settings.cpp
)

set(rosgui_cpp_HDRS
  ../../include/rosgui_cpp/plugin_bridge.h
)

set(rosgui_cpp_INCLUDE_DIRECTORIES
  ${PYTHON_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include/rosgui_cpp
  ${PROJECT_SOURCE_DIR}/src/rosgui_cpp
)

set(rosgui_cpp_LINK_LIBRARIES
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

qt4_wrap_cpp(rosgui_cpp_MOCS ${rosgui_cpp_HDRS})

include_directories(${PROJECT_NAME} ${rosgui_cpp_INCLUDE_DIRECTORIES})
rosbuild_add_library(${PROJECT_NAME} ${rosgui_cpp_SRCS} ${rosgui_cpp_MOCS})
target_link_libraries(${PROJECT_NAME} ${rosgui_cpp_LINK_LIBRARIES})

