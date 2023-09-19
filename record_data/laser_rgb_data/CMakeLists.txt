cmake_minimum_required(VERSION 3.0.2)
project(laser_rgb_data)


#add_compile_options(-std=c++11)

set  (CMAKE_BUILD_TYPE "Release")
set  (CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -static-libgcc -static-libstdc++")
set  (CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -pthread -static-libgcc -static-libstdc++")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  image_transport
        laser_geometry
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS 
  include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)



add_executable(${PROJECT_NAME}_node src/get_rgb_laser.cpp ${Boost_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME}_node
				 ${catkin_LIBRARIES} 
         ${OpenCV_LIBS}
         ${PCL_LIBRARIES} 
         ${CMAKE_CURRENT_SOURCE_DIR}/lib/libneusdk.so 
         ${CMAKE_CURRENT_SOURCE_DIR}/lib/libwoodpecker_arm.so
         ${CMAKE_CURRENT_SOURCE_DIR}/lib/libyaml-cpp.so.0.7.0
         -lopencv_imgcodecs  
         -luuid -lcurl -lboost_thread -lboost_system -lpthread )
