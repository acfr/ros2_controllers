#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "swerve_controller::swerve_controller" for configuration "None"
set_property(TARGET swerve_controller::swerve_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(swerve_controller::swerve_controller PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/libswerve_controller.so"
  IMPORTED_SONAME_NONE "libswerve_controller.so"
  )

list(APPEND _cmake_import_check_targets swerve_controller::swerve_controller )
list(APPEND _cmake_import_check_files_for_swerve_controller::swerve_controller "${_IMPORT_PREFIX}/lib/libswerve_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
