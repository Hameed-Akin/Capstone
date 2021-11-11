# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.12)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (json_reader)

# The below line will allow using C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable
add_executable(Main main.cpp)

target_link_libraries(Main jsoncpp)
