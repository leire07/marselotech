# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_marselotech_my_world_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED marselotech_my_world_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(marselotech_my_world_FOUND FALSE)
  elseif(NOT marselotech_my_world_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(marselotech_my_world_FOUND FALSE)
  endif()
  return()
endif()
set(_marselotech_my_world_CONFIG_INCLUDED TRUE)

# output package information
if(NOT marselotech_my_world_FIND_QUIETLY)
  message(STATUS "Found marselotech_my_world: 0.0.0 (${marselotech_my_world_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'marselotech_my_world' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${marselotech_my_world_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(marselotech_my_world_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${marselotech_my_world_DIR}/${_extra}")
endforeach()