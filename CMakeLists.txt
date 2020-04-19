cmake_minimum_required (VERSION 3.4)

project (serial_test)

LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

set (CMAKE_DEBUG_POSTFIX "_d")

include_directories("include")

if(WIN32)
  set(COMMON_LIBS )
elseif (UNIX) 
  set(COMMON_LIBS )
else ()
  set(COMMON_LIBS )
endif()

set(RUN_DIR ${PROJECT_SOURCE_DIR}/bin)

add_executable(serial_test main.cpp src/SerialPort.cpp)

set_property(TARGET serial_test PROPERTY DEBUG_POSTFIX _d)

target_link_libraries(serial_test ${COMMON_LIBS})

ADD_CUSTOM_TARGET(debug ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE:STRING=Debug ${PROJECT_SOURCE_DIR})
ADD_CUSTOM_TARGET(release ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE:STRING=Release ${PROJECT_SOURCE_DIR})

set_property(TARGET serial_test PROPERTY CXX_STANDARD 11)
set_property(TARGET serial_test PROPERTY CXX_STANDARD_REQUIRED ON)
