cmake_minimum_required(VERSION 3.10.0)
project(RaspberryPiBARO)
add_subdirectory(src)
add_executable(RaspberryPiBARO TestModule.cpp)
target_link_libraries(RaspberryPiBARO LIBBARO)