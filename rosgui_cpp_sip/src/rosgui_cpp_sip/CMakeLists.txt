string(REGEX REPLACE "^([^;]*;)*([^;]+/rosgui_cpp)/include(;[^;]*)*$" "\\2" rosgui_cpp_SOURCE_DIR "${rosgui_cpp_sip_INCLUDE_DIRS}")

set(SIP_BUILD_DIR ${PROJECT_SOURCE_DIR}/build/sip/rosgui_cpp_sip)
set(SIP_SRC_DIR ${PROJECT_SOURCE_DIR}/src/rosgui_cpp_sip)

set(rosgui_cpp_sip_SIPS
  composite_plugin_provider.sip
  generic_proxy.sip
  main_window_interface.sip
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

add_custom_command(
  OUTPUT ${SIP_BUILD_DIR}/Makefile
  COMMAND python configure.py ${rosgui_cpp_SOURCE_DIR} ${SIP_BUILD_DIR} rosgui_cpp.sip ${PROJECT_SOURCE_DIR}/lib
  DEPENDS configure.py ${rosgui_cpp_sip_SIPS}
  WORKING_DIRECTORY ${SIP_SRC_DIR}
  COMMENT "Running SIP generator for librosgui_cpp Python bindings..."
)

add_custom_target(librosgui_cpp_sip.so ALL
  COMMAND make
  DEPENDS ${SIP_BUILD_DIR}/Makefile
  WORKING_DIRECTORY ${SIP_BUILD_DIR}
  COMMENT "Compiling generated code for librosgui_cpp Python bindings..."
)
