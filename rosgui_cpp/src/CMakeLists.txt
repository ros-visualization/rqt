add_subdirectory(rosgui_cpp)

set(rosgui_cpp_BINDINGS "")

add_subdirectory(rosgui_cpp_shiboken)
add_subdirectory(rosgui_cpp_sip)

message("Python binding generators: ${rosgui_cpp_BINDINGS}")
if(NOT rosgui_cpp_BINDINGS)
  message(FATAL_ERROR "No Python binding generator found.")
endif()
