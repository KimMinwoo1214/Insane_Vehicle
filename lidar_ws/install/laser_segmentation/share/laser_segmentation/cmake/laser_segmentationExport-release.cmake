#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "laser_segmentation::laser_segmentation_core" for configuration "Release"
set_property(TARGET laser_segmentation::laser_segmentation_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(laser_segmentation::laser_segmentation_core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblaser_segmentation_core.so"
  IMPORTED_SONAME_RELEASE "liblaser_segmentation_core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS laser_segmentation::laser_segmentation_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_laser_segmentation::laser_segmentation_core "${_IMPORT_PREFIX}/lib/liblaser_segmentation_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
