find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
set(rosgui_cpp_shiboken_QT_COMPONENTS
  QtCore
  QtGui
)

set(rosgui_cpp_shiboken_SRCS
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

set(rosgui_cpp_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/include)
set(rosgui_cpp_HDRS
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

rosbuild_include(python_qt_binding shiboken_helper)

if(shiboken_helper_FOUND)
  list(APPEND rosgui_cpp_BINDINGS "shiboken")
  set(rosgui_cpp_BINDINGS "${rosgui_cpp_BINDINGS}" PARENT_SCOPE)

  shiboken_generator(librosgui_cpp global.h typesystem.xml ${PROJECT_SOURCE_DIR}/src/rosgui_cpp_shiboken "${rosgui_cpp_shiboken_SRCS}" "${rosgui_cpp_HDRS}" "${rosgui_cpp_INCLUDE_PATH}" "${PROJECT_SOURCE_DIR}/build/src/rosgui_cpp_shiboken")

  include_directories(rosgui_cpp_shiboken ${rosgui_cpp_INCLUDE_PATH}/rosgui_cpp)
  shiboken_include_directories(rosgui_cpp_shiboken "${rosgui_cpp_shiboken_QT_COMPONENTS}")
  rosbuild_add_library(rosgui_cpp_shiboken ${rosgui_cpp_shiboken_SRCS})
  target_link_libraries(rosgui_cpp_shiboken ${PROJECT_NAME})
  shiboken_target_link_libraries(rosgui_cpp_shiboken "${rosgui_cpp_shiboken_QT_COMPONENTS}")
endif()
