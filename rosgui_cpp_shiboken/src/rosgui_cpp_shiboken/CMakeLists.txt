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
  librosgui_cpp_shiboken/rosgui_cpp_mainwindowinterface_wrapper.cpp
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

string(REGEX MATCH "^.*/rosgui_cpp/include" rosgui_cpp_INCLUDE_PATH "${rosgui_cpp_shiboken_INCLUDE_DIRS}")

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

add_custom_command(
  OUTPUT ${rosgui_cpp_shiboken_SRCS}
  COMMAND ${GENERATORRUNNER_BINARY}
  --generatorSet=shiboken
  ${PROJECT_SOURCE_DIR}/src/rosgui_cpp_shiboken/global.h
  --include-paths=${rosgui_cpp_INCLUDE_PATH}:${QT_INCLUDE_DIR}:/usr/include
  --typesystem-paths=${PYSIDE_TYPESYSTEMS}
  --output-directory=${PROJECT_SOURCE_DIR}/build/src/rosgui_cpp_shiboken
  ${PROJECT_SOURCE_DIR}/src/rosgui_cpp_shiboken/typesystem.xml
  COMMENT "Running Shiboken generator for librosgui_cpp Python bindings..."
)

include_directories(${PROJECT_NAME} ${rosgui_cpp_shiboken_INCLUDE_DIRECTORIES})
rosbuild_add_library(${PROJECT_NAME} ${rosgui_cpp_shiboken_SRCS})
target_link_libraries(${PROJECT_NAME} ${rosgui_cpp_shiboken_LINK_LIBRARIES})

