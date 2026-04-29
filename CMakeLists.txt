cmake_minimum_required(VERSION 3.8)

project(testing)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_executable(testing "main.cpp")
add_definitions(-DJPH_OBJECT_STREAM)
target_include_directories(testing PRIVATE "C:/lib/include")
target_include_directories(testing PRIVATE "include")
target_include_directories(testing PRIVATE "../lib/include")
target_link_directories(testing PRIVATE "C:/lib/lib")
target_link_libraries(testing jolt raylib winmm)
