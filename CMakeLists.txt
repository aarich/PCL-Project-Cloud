cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(image_projection)
 
find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )

 
include_directories(${PCL_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB image_projection
    "src/*.h"
    "src/*.cpp"
)
add_executable(image_projection ${image_projection})
 
target_link_libraries (image_projection ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
