cmake_minimum_required(VERSION 3.28)
project(fly_simulator)

set(CMAKE_CXX_STANDARD 23)

find_package(SDL2)
find_package(Eigen3)
find_package(cxxopts)

add_executable(fly_simulator main.cpp)
target_link_libraries(fly_simulator SDL2::SDL2 Eigen3::Eigen cxxopts::cxxopts)

target_compile_options(fly_simulator PUBLIC
    $<$<NOT:$<CONFIG:Debug>>:-march=x86-64-v4>
)
