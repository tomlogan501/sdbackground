cmake_minimum_required(VERSION 2.8.3)
project(sdbackground)

## Compile as C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    image_transport
    dynamic_reconfigure
    roslint
    )

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    APPEND_COVERAGE_COMPILER_FLAGS()
endif()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/sdbg.cfg
    )

###########
## Build ##
###########

## Create lint target
roslint_cpp()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    # include
    test/unit/
    src/extern/
    src
    ${catkin_INCLUDE_DIRS}
    )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
    src/sdbackground_node.cpp
    src/extern/SD_basics.c
    src/extern/nrutil.c
    )

add_library(${PROJECT_NAME}
    src/extern/SD_basics.c
    src/extern/nrutil.c
    )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )

#############
## Install ##
#############

## Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
    launch/webcam_test.launch
    launch/webcam_test2.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

#############
## Testing ##
#############

## copy dataset to build
#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/dataset
#    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test/unit)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)

    # Create a target ${PROJECT_NAME}_coverage_report
    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*"
            "*/${PROJECT_NAME}/build/*")
        add_code_coverage(
            NAME ${PROJECT_NAME}_coverage_report
            DEPENDENCIES tests
            )
    endif()

    catkin_add_gtest(${PROJECT_NAME}_test
        test/unit/test_sdbackground.cpp
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/unit)

    target_link_libraries(  ${PROJECT_NAME}_test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES})
endif()
