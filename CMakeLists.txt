cmake_minimum_required(VERSION 3.16)
project(sads_fsw LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(sads_fsw
    src/main.cpp
    src/imu_task.cpp
    src/estimator_task.cpp
    src/control_task.cpp
    src/comm_task.cpp
    src/server_task.cpp
    src/mti03_driver.cpp
    src/i2c_rw.cpp
)

target_include_directories(sads_fsw PRIVATE src)

find_package(Threads REQUIRED)
target_link_libraries(sads_fsw PRIVATE Threads::Threads)
