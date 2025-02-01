cmake_minimum_required(VERSION 3.8)
project(wu25)

set(CMAKE_CXX_STANDARD 23)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if ("${DUMMY}" STREQUAL "True")
    message(VERBOSE "Compiling dummy libraries")
    set(DUMMY ON)
endif ()

file(GLOB_RECURSE SRC_DIR "src/dummy/*.c")
set_source_files_properties(
        ${SRC_DIR}
        PROPERTIES
        COMPILE_FLAGS "-Wno-all"
)

include_directories(include deps/sh2 deps/eigen deps/csv)

add_subdirectory(deps/matplotplusplus)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(sh2lib deps/sh2/euler.c deps/sh2/sh2_SensorValue.c deps/sh2/sh2.c deps/sh2/sh2_util.c deps/sh2/shtp.c)

if (DUMMY)
    add_library(dummy_src src/dummy/wiringPi.c)
endif ()

add_executable(bno_node src/bno_node.cpp include/bno_node.hpp include/drivers/bno08x.hpp include/drivers/spi.hpp src/drivers/bno08x.cpp src/drivers/spi.cpp)
add_executable(thrusters
        src/thruster_node.cpp
        src/thrusters/thrusters.cpp
        src/thrusters/thruster_data.cpp
        include/thrusters/thrusters.hpp
        include/util/PolynomialRegression.hpp
        include/util/controller_map.hpp
        include/util/quaternion_pid.hpp
        deps/csv/csv.hpp
        src/util/quaternion_pid.cpp)

target_link_libraries(bno_node sh2lib gpiod)
target_link_libraries(thrusters Eigen3::Eigen matplot)

if (NOT DUMMY)
    target_link_libraries(bno_node wiringPi)
else ()
    target_link_libraries(bno_node dummy_src)
endif ()

target_include_directories(bno_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bno_node rclcpp std_msgs)
ament_target_dependencies(thrusters rclcpp std_msgs sensor_msgs)

install(TARGETS
        sh2lib
        bno_node
        thrusters
        DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(scripts)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
