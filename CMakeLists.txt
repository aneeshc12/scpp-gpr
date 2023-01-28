cmake_minimum_required(VERSION 2.8.3)
project(test_gpr)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set(CMAKE_CXX_STANDARD 14)

# find_package(catkin REQUIRED COMPONENTS
#   tf
#   roscpp
#   rospy
#   cv_bridge
#   image_transport

#   pcl_ros
#   pcl_conversions

# )

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)


include_directories(
	include
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

# add_executable(imageProjection src/imageProjection.cpp)
# add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
# target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(featureAssociation src/featureAssociation.cpp)
# add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
# target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(testGPR 
    src/testGPR.cpp
    src/Scancontext.cpp
)
target_link_libraries(testGPR 
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    Eigen3::Eigen
)
