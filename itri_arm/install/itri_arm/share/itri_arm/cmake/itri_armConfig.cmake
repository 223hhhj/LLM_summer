# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_itri_arm_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED itri_arm_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(itri_arm_FOUND FALSE)
  elseif(NOT itri_arm_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(itri_arm_FOUND FALSE)
  endif()
  return()
endif()
set(_itri_arm_CONFIG_INCLUDED TRUE)

# output package information
if(NOT itri_arm_FIND_QUIETLY)
  message(STATUS "Found itri_arm: 0.0.0 (${itri_arm_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'itri_arm' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${itri_arm_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(itri_arm_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${itri_arm_DIR}/${_extra}")
endforeach()
