cmake_minimum_required(VERSION 2.8.3)
project(dvs_displayer)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# Make the executable
cs_add_executable(${PROJECT_NAME}
  src/custom_cmaps.cpp
  src/displayer.cpp
  src/displayer_node.cpp
)

# Link the executable to the necessary libs
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()
cs_export()
