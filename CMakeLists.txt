cmake_minimum_required(VERSION 3.0.0)
project(auto_diff_on_manifold VERSION 0.1.0)


set(CMAKE_BUILD_TYPE Release )
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -g -mavx")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -O3 -pthread")


### Packages 
find_package( Ceres REQUIRED)
find_package(Pangolin REQUIRED )

### Include 
include_directories( ${CERES_INCLUDE_DIRS}  ${Pangolin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src )

file(GLOB SRC_FILES
    "src/*.cc")
list(APPEND SRC_FILES  src/viewer/viewer.cc)

add_executable( pose_estimation_test
     ${SRC_FILES} )
target_link_libraries(pose_estimation_test
     ${CERES_LIBRARIES}
     ${Pangolin_LIBRARIES}
     )
