set(rosgui_cpp_HDRS_DIR ${CMAKE_SOURCE_DIR}/include/rosgui_cpp)
set(rosgui_cpp_sip_DEPENDENT_FILES
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

# ugly hack to obtain path to shared sip_configure.py
string(REGEX REPLACE "^([^;]*;)*([^;]+/python_qt_binding)/include(;[^;]*)*$" "\\2" python_qt_binding_SOURCE_DIR "${rosgui_cpp_INCLUDE_DIRS}")

# maintain context for different named target
set(rosgui_cpp_sip_INCLUDE_DIRS ${rosgui_cpp_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
set(rosgui_cpp_sip_LIBRARIES ${rosgui_cpp_LIBRARIES} ${PROJECT_NAME})
set(rosgui_cpp_sip_LIBRARY_DIRS ${rosgui_cpp_LIBRARY_DIRS} ${CMAKE_SOURCE_DIR}/lib)
set(rosgui_cpp_sip_LDFLAGS_OTHER ${rosgui_cpp_LDFLAGS_OTHER} -Wl,-rpath,${CMAKE_SOURCE_DIR}/lib)

rosbuild_include(python_qt_binding sip_helper)

if(sip_helper_FOUND)
  list(APPEND rosgui_cpp_BINDINGS "sip")
  set(rosgui_cpp_BINDINGS "${rosgui_cpp_BINDINGS}" PARENT_SCOPE)

  build_sip_binding(rosgui_cpp_sip rosgui_cpp.sip
    SIP_CONFIGURE ${python_qt_binding_SOURCE_DIR}/cmake/sip_configure.py
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/rosgui_cpp_sip
    DEPENDS ${rosgui_cpp_sip_DEPENDENT_FILES}
    DEPENDENCIES rosgui_cpp
  )
endif()
