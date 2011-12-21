find_package(GeneratorRunner REQUIRED)
find_package(Shiboken REQUIRED)
find_package(PySide REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

SET(rosgui_cpp_shiboken_SRCS
  librosgui_cpp_shiboken/librosgui_cpp_shiboken_module_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_compositepluginprovider_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_genericproxy_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_plugin_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_pluginbridge_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_plugincontext_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_plugindescriptor_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_pluginprovider_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_recursivepluginprovider_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_rospluginlibpluginprovider_forpluginproviders_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_rospluginlibpluginprovider_forplugins_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_settings_wrapper.cpp
  librosgui_cpp_shiboken/rosgui_cpp_wrapper.cpp
)

string(REGEX REPLACE "^([^;]*;)*([^;]+/rosgui_cpp/include)(;[^;]*)*$" "\\2" rosgui_cpp_INCLUDE_PATH "${rosgui_cpp_shiboken_INCLUDE_DIRS}")

SET(rosgui_cpp_HDRS
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/composite_plugin_provider.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/generic_proxy.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/plugin.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/plugin_bridge.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/plugin_context.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/plugin_descriptor.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/plugin_provider.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/recursive_plugin_provider.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/ros_pluginlib_plugin_provider.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/ros_pluginlib_plugin_provider_for_plugin_providers.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/ros_pluginlib_plugin_provider_for_plugins.h
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp/settings.h
)

set(rosgui_cpp_shiboken_INCLUDE_DIRECTORIES
  ${PYTHON_INCLUDE_DIR}
  ${SHIBOKEN_INCLUDE_DIR}
  ${PYSIDE_INCLUDE_DIR}
  ${PYSIDE_INCLUDE_DIR}/QtCore
  ${PYSIDE_INCLUDE_DIR}/QtGui
  ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp
)

set(rosgui_cpp_shiboken_LINK_LIBRARIES
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
  ${SHIBOKEN_PYTHON_LIBRARIES}
  ${SHIBOKEN_LIBRARY}
  ${PYSIDE_LIBRARY}
)

# See ticket https://code.ros.org/trac/ros-pkg/ticket/5219
set(QT_INCLUDE_DIR_WITH_COLONS "")
foreach(dir ${QT_INCLUDE_DIR})
  set(QT_INCLUDE_DIR_WITH_COLONS "${QT_INCLUDE_DIR_WITH_COLONS}:${dir}")
endforeach()

add_custom_command(
  OUTPUT ${rosgui_cpp_shiboken_SRCS}
  COMMAND ${GENERATORRUNNER_BINARY}
    --generatorSet=shiboken
    --include-paths=${rosgui_cpp_INCLUDE_PATH}:${QT_INCLUDE_DIR_WITH_COLONS}
    --typesystem-paths=${PYSIDE_TYPESYSTEMS}
    --output-directory=${PROJECT_SOURCE_DIR}/build/src/rosgui_cpp_shiboken
    global.h
    typesystem.xml
  DEPENDS global.h typesystem.xml ${rosgui_cpp_HDRS}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/rosgui_cpp_shiboken
  COMMENT "Running Shiboken generator for librosgui_cpp Python bindings..."
)

include_directories(${PROJECT_NAME} ${rosgui_cpp_shiboken_INCLUDE_DIRECTORIES})
rosbuild_add_library(${PROJECT_NAME} ${rosgui_cpp_shiboken_SRCS})
target_link_libraries(${PROJECT_NAME} ${rosgui_cpp_shiboken_LINK_LIBRARIES})

