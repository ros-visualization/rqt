string(REGEX REPLACE "^([^;]*;)*([^;]+/python_qt_binding)/include(;[^;]*)*$" "\\2" python_qt_binding_SOURCE_DIR "${rosgui_cpp_sip_INCLUDE_DIRS}")
include(${python_qt_binding_SOURCE_DIR}/cmake/sip_helper.cmake)

set(rosgui_cpp_sip_DEPENDEND_SIPS
  composite_plugin_provider.sip
  generic_proxy.sip
  plugin.sip
  plugin_bridge.sip
  plugin_context.sip
  plugin_descriptor.sip
  plugin_provider.sip
  recursive_plugin_provider.sip
  rosgui_cpp.sip
  ros_pluginlib_plugin_provider_for_plugin_providers.sip
  ros_pluginlib_plugin_provider_for_plugins.sip
  settings.sip
)

string(REGEX REPLACE "^([^;]*;)*([^;]+/rosgui_cpp)/include(;[^;]*)*$" "\\2" rosgui_cpp_SOURCE_DIR "${rosgui_cpp_sip_INCLUDE_DIRS}")
set(rosgui_cpp_HDRS_DIR ${rosgui_cpp_SOURCE_DIR}/include/rosgui_cpp)
set(rosgui_cpp_HDRS
  ${rosgui_cpp_HDRS_DIR}/composite_plugin_provider.h
  ${rosgui_cpp_HDRS_DIR}/generic_proxy.h
  ${rosgui_cpp_HDRS_DIR}/plugin.h
  ${rosgui_cpp_HDRS_DIR}/plugin_bridge.h
  ${rosgui_cpp_HDRS_DIR}/plugin_context.h
  ${rosgui_cpp_HDRS_DIR}/plugin_descriptor.h
  ${rosgui_cpp_HDRS_DIR}/plugin_provider.h
  ${rosgui_cpp_HDRS_DIR}/recursive_plugin_provider.h
  ${rosgui_cpp_HDRS_DIR}/ros_pluginlib_plugin_provider.h
  ${rosgui_cpp_HDRS_DIR}/ros_pluginlib_plugin_provider_for_plugin_providers.h
  ${rosgui_cpp_HDRS_DIR}/ros_pluginlib_plugin_provider_for_plugins.h
  ${rosgui_cpp_HDRS_DIR}/settings.h
)

build_sip_binding(${PROJECT_NAME} ${python_qt_binding_SOURCE_DIR}/cmake/sip_configure.py rosgui_cpp.sip "${rosgui_cpp_sip_DEPENDEND_SIPS}" "${rosgui_cpp_HDRS}" ${PROJECT_SOURCE_DIR}/src/rosgui_cpp_sip ${PROJECT_SOURCE_DIR}/lib ${PROJECT_BINARY_DIR})
