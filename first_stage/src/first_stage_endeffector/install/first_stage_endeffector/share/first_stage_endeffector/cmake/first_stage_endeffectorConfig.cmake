# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_first_stage_endeffector_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED first_stage_endeffector_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(first_stage_endeffector_FOUND FALSE)
  elseif(NOT first_stage_endeffector_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(first_stage_endeffector_FOUND FALSE)
  endif()
  return()
endif()
set(_first_stage_endeffector_CONFIG_INCLUDED TRUE)

# output package information
if(NOT first_stage_endeffector_FIND_QUIETLY)
  message(STATUS "Found first_stage_endeffector: 0.0.0 (${first_stage_endeffector_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'first_stage_endeffector' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${first_stage_endeffector_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(first_stage_endeffector_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${first_stage_endeffector_DIR}/${_extra}")
endforeach()
