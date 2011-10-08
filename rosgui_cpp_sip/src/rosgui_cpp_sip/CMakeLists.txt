string(REGEX REPLACE "^([^;]*;)*([^;]+/rosgui_cpp)/include(;[^;]*)*$" "\\2" rosgui_cpp_SOURCE_DIR "${rosgui_cpp_sip_INCLUDE_DIRS}")

set(SIP_BUILD_DIR build/sip/rosgui_cpp_sip)
set(SIP_SRC_DIR ${PROJECT_SOURCE_DIR}/src/rosgui_cpp_sip)

add_custom_target(librosgui_cpp_sip.so ALL
  SOURCES ${SIP_SRC_DIR}/configure.py ${SIP_SRC_DIR}/rosgui_cpp.sip
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMAND python ${SIP_SRC_DIR}/configure.py ${rosgui_cpp_SOURCE_DIR} ${SIP_BUILD_DIR} ${SIP_SRC_DIR}/rosgui_cpp.sip
  COMMAND make --directory=${SIP_BUILD_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/lib
  COMMAND ${CMAKE_COMMAND} -E copy ${SIP_BUILD_DIR}/librosgui_cpp_sip.so ${PROJECT_SOURCE_DIR}/lib/
  COMMENT "Running SIP generator for librosgui_cpp Python bindings..."
)
