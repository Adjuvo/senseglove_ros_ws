# CMake function to add an rpath (runtime search path) to a given target
# Ensures that the target executable can find its dependent libraries at runtime.
# Copied from https://github.com/shadow-robot/ros_ethercat/blob/kinetic-devel/ros_ethercat_hardware/cmake/ros_ethercat_hardware-extras.cmake.em

function(ros_enable_rpath target)

   # Set ${target} with RPATH built in so that we can install it
   set_target_properties(${target} PROPERTIES SKIP_BUILD_RPATH FALSE)

   # Set the install 'RPATH' to hold the install path where the libraries will be installed
   set(RPATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}")

   # If LD_LIBRARY_PATH is set, add it to the install RPATH
   #  this works in a normal catkin environment, but fails if the user unsets
   #  their LD_LIBRARY_PATH manually for some reason
   if(DEFINED ENV{LD_LIBRARY_PATH})
      set(RPATH "${RPATH}:$ENV{LD_LIBRARY_PATH}")
   endif()

   # Apply our computed RPATH to the target
   set_target_properties(${target} PROPERTIES INSTALL_RPATH ${RPATH})

   # Don't use the final RPATH in devel space
   set_target_properties(${target} PROPERTIES BUILD_WITH_INSTALL_RPATH FALSE)
endfunction()
