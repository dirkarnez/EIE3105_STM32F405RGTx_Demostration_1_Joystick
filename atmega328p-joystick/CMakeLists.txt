cmake_minimum_required(VERSION 3.24)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project("EIE3105_ATmega328P_Application")

set_source_files_properties("main.c" PROPERTIES LANGUAGE CXX)

enable_testing()

include(CTest)
include(GoogleTest)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")

find_package(GTest 1.14.0 EXACT REQUIRED)

if(GTest_FOUND)
    message(STATUS "GTest FOUND")
else()
    message(STATUS "GTest NOT FOUND")
endif()

add_executable(${PROJECT_NAME} "main.c")

target_link_libraries(
  ${PROJECT_NAME}
  GTest::gtest
  GTest::gtest_main
)

gtest_discover_tests(${PROJECT_NAME})