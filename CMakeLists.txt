cmake_minimum_required(VERSION 3.15)

project(vecrender)
add_library(vecrender
    vecrender_mathutils.cpp
    vecrender_path.cpp
    vecrender_triangulator.cpp
    vecrender_localtriangulator.cpp
    vecrender_domaintrianglemarker.cpp
)

option(VECRENDER_TRIANGULATION_DEBUG "Debug triangulation" OFF)
if (VECRENDER_TRIANGULATION_DEBUG)
    target_compile_definitions(vecrender PUBLIC VECRENDER_TRIANGULATION_DEBUG)
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL GNU OR
        CMAKE_CXX_COMPILER_ID STREQUAL AppleClang OR
        CMAKE_CXX_COMPILER_ID STREQUAL Clang)
    target_compile_options(vecrender PRIVATE -Wall -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL MSVC)
    target_compile_options(vecrender PRIVATE /W3)
endif()

set_property(TARGET vecrender PROPERTY CXX_STANDARD 20)
target_include_directories(vecrender PRIVATE .)

find_package(glm REQUIRED)
target_link_libraries(vecrender PUBLIC glm)

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE ON)
find_package(CGAL REQUIRED)
target_link_libraries(vecrender PRIVATE CGAL)
