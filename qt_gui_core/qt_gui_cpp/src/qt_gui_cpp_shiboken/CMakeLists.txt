find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
set(qt_gui_cpp_shiboken_QT_COMPONENTS
  QtCore
  QtGui
)

set(qt_gui_cpp_shiboken_SRCS
  libqt_gui_cpp_shiboken/libqt_gui_cpp_shiboken_module_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_compositepluginprovider_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_genericproxy_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_plugin_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_pluginbridge_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_plugincontext_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_plugindescriptor_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_pluginprovider_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_recursivepluginprovider_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_rospluginlibpluginprovider_forpluginproviders_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_rospluginlibpluginprovider_forplugins_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_settings_wrapper.cpp
  libqt_gui_cpp_shiboken/qt_gui_cpp_wrapper.cpp
)

set(qt_gui_cpp_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/include)
set(qt_gui_cpp_HDRS
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/composite_plugin_provider.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/generic_proxy.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/plugin.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/plugin_bridge.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/plugin_context.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/plugin_descriptor.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/plugin_provider.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/recursive_plugin_provider.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/ros_pluginlib_plugin_provider.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/ros_pluginlib_plugin_provider_for_plugin_providers.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/ros_pluginlib_plugin_provider_for_plugins.h
  ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp/settings.h
)

rosbuild_include(python_qt_binding shiboken_helper)

if(shiboken_helper_FOUND)
  rosbuild_include(python_qt_binding shiboken_check_qgenericreturnargument)

  shiboken_check_qgenericreturnargument(SHIBOKEN_SUPPORTS_QGENERICRETURNARGUMENT)
  if(${SHIBOKEN_SUPPORTS_QGENERICRETURNARGUMENT})
    list(APPEND qt_gui_cpp_BINDINGS "shiboken")
    set(qt_gui_cpp_BINDINGS "${qt_gui_cpp_BINDINGS}" PARENT_SCOPE)

    shiboken_generator(libqt_gui_cpp global.h typesystem.xml ${PROJECT_SOURCE_DIR}/src/qt_gui_cpp_shiboken "${qt_gui_cpp_shiboken_SRCS}" "${qt_gui_cpp_HDRS}" "${qt_gui_cpp_INCLUDE_PATH}" "${PROJECT_SOURCE_DIR}/build/src/qt_gui_cpp_shiboken")

    include_directories(qt_gui_cpp_shiboken ${qt_gui_cpp_INCLUDE_PATH}/qt_gui_cpp)
    shiboken_include_directories(qt_gui_cpp_shiboken "${qt_gui_cpp_shiboken_QT_COMPONENTS}")
    rosbuild_add_library(qt_gui_cpp_shiboken ${qt_gui_cpp_shiboken_SRCS})
    target_link_libraries(qt_gui_cpp_shiboken ${PROJECT_NAME})
    shiboken_target_link_libraries(qt_gui_cpp_shiboken "${qt_gui_cpp_shiboken_QT_COMPONENTS}")
  else()
    message(WARNING "Shiboken does not support QGenericReturnArgument - skipped building bindings")
  endif()
endif()
