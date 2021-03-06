cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/CovarianceTimeCache.cpp src/UncertainTransformBroadcaster.cpp  src/UncertainTransformer.cpp src/UncertainTransformListener.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(tutorial src/tutorial.cpp)
target_link_libraries(tutorial ${PROJECT_NAME})
rosbuild_add_executable(static_covariance_publisher src/static_covariance_publisher.cpp)
target_link_libraries(static_covariance_publisher ${PROJECT_NAME})
rosbuild_add_executable(static_variance_publisher src/static_variance_publisher.cpp)
target_link_libraries(static_variance_publisher ${PROJECT_NAME})
rosbuild_add_executable(visualize_samples src/visualize_samples.cpp)
target_link_libraries(visualize_samples ${PROJECT_NAME})
rosbuild_add_executable(example_tf_tree src/example_tf_tree.cpp)
target_link_libraries(example_tf_tree ${PROJECT_NAME})
rosbuild_add_executable(time_travel src/time_travel.cpp)
target_link_libraries(time_travel ${PROJECT_NAME})
#rosbuild_add_executable(eigen_playground src/eigen_playground.cpp)
#target_link_libraries(example ${PROJECT_NAME})
