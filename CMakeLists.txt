cmake_minimum_required(VERSION 3.10)
project(rm_common)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-enum-compare)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslint

        tf
        rm_msgs
        geometry_msgs
        realtime_tools
        )

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIR}
        CATKIN_DEPENDS
        rm_msgs
        roscpp
        DEPENDS
        LIBRARIES
        rm_common
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        include
)

file(GLOB_RECURSE sources "src/*.cpp")

add_library(rm_common SHARED ${sources})
add_executable(test_traj test/test_traj.cpp)
add_executable(test_kalman test/test_kalman_filter.cpp)

target_link_libraries(rm_common ${catkin_LIBRARIES})
target_link_libraries(test_traj rm_common ${catkin_LIBRARIES})
target_link_libraries(test_kalman rm_common ${catkin_LIBRARIES})

#############
## Testing ##
#############

if (${CATKIN_ENABLE_TESTING})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    ## Add gtest based cpp test target and link libraries
    catkin_add_gtest(lqr_test
            test/unit_test_lqr.cpp
            test/LqrTest.cpp)
    target_link_libraries(lqr_test rm_common)
endif ()

roslint_cpp()
