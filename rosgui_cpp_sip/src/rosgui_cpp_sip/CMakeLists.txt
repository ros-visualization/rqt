string(REGEX REPLACE "^([^;]*;)*([^;]+/rosgui_cpp)/include(;[^;]*)*$" "\\2" rosgui_cpp_SOURCE_DIR "${rosgui_cpp_sip_INCLUDE_DIRS}")

set(rosgui_cpp_HDRS_DIR ${rosgui_cpp_SOURCE_DIR}/include/rosgui_cpp)
set(SIP_BUILD_DIR ${PROJECT_SOURCE_DIR}/build/sip/rosgui_cpp_sip)
set(SIP_SRC_DIR ${PROJECT_SOURCE_DIR}/src/rosgui_cpp_sip)

set(rosgui_cpp_sip_SIPS
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

add_custom_command(
  OUTPUT ${SIP_BUILD_DIR}/Makefile
  COMMAND python configure.py ${SIP_BUILD_DIR} rosgui_cpp.sip ${PROJECT_SOURCE_DIR}/lib
  DEPENDS configure.py ${rosgui_cpp_sip_SIPS} ${rosgui_cpp_HDRS}
  WORKING_DIRECTORY ${SIP_SRC_DIR}
  COMMENT "Running SIP generator for librosgui_cpp Python bindings..."
)

add_custom_target(librosgui_cpp_sip.so ALL
  COMMAND make
  DEPENDS ${SIP_BUILD_DIR}/Makefile
  WORKING_DIRECTORY ${SIP_BUILD_DIR}
  COMMENT "Compiling generated code for librosgui_cpp Python bindings..."
)
