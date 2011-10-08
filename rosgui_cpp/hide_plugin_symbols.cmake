# hides the symbols of a Poco plugin
macro(rosgui_cpp__hide_plugin_symbols package_name class_name)
  set(_version_script version__${package_name}__${class_name}.script)
  FILE(WRITE ${CMAKE_BINARY_DIR}/${_version_script}
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
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-version-script=${CMAKE_BINARY_DIR}/${_version_script}")
endmacro()

