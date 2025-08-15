#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "itri_arm::itri_arm_hardware" for configuration ""
set_property(TARGET itri_arm::itri_arm_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(itri_arm::itri_arm_hardware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libitri_arm_hardware.so"
  IMPORTED_SONAME_NOCONFIG "libitri_arm_hardware.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS itri_arm::itri_arm_hardware )
list(APPEND _IMPORT_CHECK_FILES_FOR_itri_arm::itri_arm_hardware "${_IMPORT_PREFIX}/lib/libitri_arm_hardware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
