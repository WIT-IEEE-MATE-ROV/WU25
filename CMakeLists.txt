cmake_minimum_required(VERSION 3.8)
project(wu25)

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

include_directories(include/wu25)
include_directories(sh2 eigen csv)

add_subdirectory(matplotplusplus)

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


add_library(bno08x src/drivers/bno08x.cpp include/wu25/bno08x.hpp)

add_library(sh2lib sh2/euler.c sh2/sh2_SensorValue.c sh2/sh2.c sh2/sh2_util.c sh2/shtp.c)

if (DUMMY)
    add_library(dummy_src src/dummy/wiringPi.c)
endif ()

add_executable(talker src/publisher_lambda_function.cpp src/dummy/wiringPi.c)
add_executable(listener src/subscriber_lambda_function.cpp)
add_executable(bno_node src/bno_node.cpp include/wu25/bno_node.hpp include/wu25/bno08x.hpp include/wu25/spi.hpp src/drivers/bno08x.cpp src/drivers/spi.cpp)
add_executable(thrusters src/thruster_node.cpp src/thrusters.cpp include/wu25/thrusters.hpp
        include/wu25/thruster_data.hpp
        src/thruster_data.cpp
        csv/csv.hpp
        include/wu25/PolynomialRegression.hpp)

target_link_libraries(talker Eigen3::Eigen libgpiod.so)
target_link_libraries(bno_node sh2lib bno08x libgpiod.so)
target_link_libraries(thrusters Eigen3::Eigen matplot)

if (NOT DUMMY)
    target_link_libraries(talker libwiringPi.so)
    target_link_libraries(bno_node libwiringPi.so)
else ()
    target_link_libraries(talker dummy_src)
    target_link_libraries(bno_node dummy_src)
endif ()

target_include_directories(bno_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(talker rclcpp std_msgs tf2 tf2_ros)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(bno_node rclcpp std_msgs)
ament_target_dependencies(thrusters rclcpp std_msgs)

install(TARGETS
        talker
        listener
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
