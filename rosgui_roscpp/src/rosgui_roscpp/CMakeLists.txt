find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

SET(rosgui_roscpp_SRCS
  nodelet_plugin_provider.cpp
  roscpp_plugin_provider.cpp
)

rosbuild_add_library(${PROJECT_NAME} ${rosgui_roscpp_SRCS})
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

if(APPLE) 
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()

