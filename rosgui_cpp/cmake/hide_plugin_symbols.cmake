# hides the symbols of a Poco plugin
macro(rosgui_cpp__hide_plugin_symbols package_name class_name)
  set(_version_script version__${package_name}__${class_name}.script)
  file(WRITE "${CMAKE_BINARY_DIR}/${_version_script}"
    "    {
      global:
        pocoBuildManifest;
        pocoInitializeLibrary;
        pocoUninitializeLibrary;
        pocoBuildManifest${package_name}__${class_name};
      local:
        *;
    };"
  )
  # checks if the linker supports version script
  include(${CMAKE_ROOT}/Modules/TestCXXAcceptsFlag.cmake)
  check_cxx_accepts_flag("-Wl,--version-script,\"${CMAKE_BINARY_DIR}/${_version_script}\"" LD_ACCEPTS_VERSION_SCRIPT)
  if(LD_ACCEPTS_VERSION_SCRIPT)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-version-script=\"${CMAKE_BINARY_DIR}/${_version_script}\"")
  endif()
endmacro()

