file(REMOVE_RECURSE
  "include/swerve_controller/swerve_controller_parameters.hpp"
  "include/swerve_controller_parameters.hpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/swerve_controller_parameters.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
