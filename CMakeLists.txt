cmake_minimum_required(VERSION 3.15...3.30)

project(sense_hat_lib VERSION 1.0
    DESCRIPTION "Sense hat utilities with pure C/C++"
    LANGUAGES CXX)

set(CMAKE_VERBOSE_MAKEFILE ON CACHE INTERNAL "to have verbose make output")

include(CTest)
add_subdirectory(src bin)
add_subdirectory(sensehat)
add_subdirectory(test)
