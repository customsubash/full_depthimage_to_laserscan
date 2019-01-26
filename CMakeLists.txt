# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8)
project(full_depthimage_to_laserscan)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED dynamic_reconfigure image_geometry image_transport nodelet roscpp sensor_msgs)
#find_package(OpenCV REQUIRED)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Depth.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES FullDepthImageToLaserScan FullDepthImageToLaserScanROS FullDepthImageToLaserScanNodelet
  CATKIN_DEPENDS dynamic_reconfigure image_geometry image_transport nodelet roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(FullDepthImageToLaserScan src/DepthImageToLaserScan.cpp)
target_link_libraries(FullDepthImageToLaserScan ${catkin_LIBRARIES})
target_compile_options(FullDepthImageToLaserScan PRIVATE -Wall -fopt-info-vec-optimized -ftree-vectorize  -ffast-math )
target_compile_options(FullDepthImageToLaserScan PUBLIC -std=c++11)


add_library(FullDepthImageToLaserScanROS src/DepthImageToLaserScanROS.cpp)
add_dependencies(FullDepthImageToLaserScanROS ${PROJECT_NAME}_gencfg)
target_link_libraries(FullDepthImageToLaserScanROS FullDepthImageToLaserScan ${catkin_LIBRARIES})

add_library(FullDepthImageToLaserScanNodelet src/DepthImageToLaserScanNodelet.cpp)
target_link_libraries(FullDepthImageToLaserScanNodelet FullDepthImageToLaserScanROS ${catkin_LIBRARIES})

add_executable(full_depthimage_to_laserscan src/depthimage_to_laserscan.cpp)
target_link_libraries(full_depthimage_to_laserscan FullDepthImageToLaserScanROS ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # Test the library
  catkin_add_gtest(libtest test/DepthImageToLaserScanTest.cpp)
  target_link_libraries(libtest DepthImageToLaserScan ${catkin_LIBRARIES})
endif()

# add the test executable, keep it from being built by "make all"
add_executable(test_dtl EXCLUDE_FROM_ALL test/depthimage_to_laserscan_rostest.cpp)

# Install targets
install(TARGETS FullDepthImageToLaserScan FullDepthImageToLaserScanROS FullDepthImageToLaserScanNodelet full_depthimage_to_laserscan
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
