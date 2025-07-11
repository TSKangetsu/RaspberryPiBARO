cmake_minimum_required(VERSION 3.10.0)
project(RaspberryPiBARO)
set(FULLSTATIC TRUE CACHE  INTERNAL bool)

add_definitions(-DDEBUG)
IF(FULLSTATIC)
  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_EXE_LINKER_FLAGS "-static -static-libstdc++ -static-libgcc")
  set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
ENDIF()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstack-usage=1000 -Os -s") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstack-usage=1000 -Os -s") 

add_subdirectory(src)
add_executable(RaspberryPiBARO TestModule.cpp)
target_link_libraries(RaspberryPiBARO LIBBARO)
target_link_libraries(RaspberryPiBARO gcc_pic)