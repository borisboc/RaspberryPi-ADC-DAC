# - Try to find libbcm2835
# Once done this will define
#  BCM2835_FOUND - System has libbcm2835
#  BCM2835_INCLUDE_DIRS - The libbcm2835 include directories
#  BCM2835_LIBRARIES - The libraries needed to use libbcm2835

if (BCM2835_INCLUDE_DIRS)
  set(BCM2835_FOUND TRUE)
else (BCM2835_INCLUDE_DIRS)

  find_path(BCM2835_INCLUDE_DIR
    NAMES
        "bcm2835.h"
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
  )

  find_library(BCM2835_LIBRARY
    NAMES
        bcm2835
        libbcm2835
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
  )
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(BCM2835 DEFAULT_MSG
                                    BCM2835_LIBRARY BCM2835_INCLUDE_DIR)

  mark_as_advanced(BCM2835_INCLUDE_DIR BCM2835_LIBRARY )

  set(BCM2835_INCLUDE_DIRS ${BCM2835_INCLUDE_DIR})
  set(BCM2835_LIBRARIES ${BCM2835_LIBRARY} )

endif (BCM2835_INCLUDE_DIRS)

