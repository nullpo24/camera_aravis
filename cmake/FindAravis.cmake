INCLUDE(FindPackageHandleStandardArgs)

FIND_PATH(aruco_INCLUDE_PATH opencv_aruco.h
  "$ENV{aruco_INCLUDE_PATH}"
  /usr/local/include/opencv_aruco
)

FIND_LIBRARY(aruco_LIBRARY opencv_aruco
  "$ENV{aruco_LIBRARY}"
  /usr/local/lib
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(aruco DEFAULT_MSG
  aruco_INCLUDE_PATH
  aruco_LIBRARY)

