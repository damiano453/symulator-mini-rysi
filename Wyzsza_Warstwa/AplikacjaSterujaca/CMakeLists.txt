cmake_minimum_required(VERSION 2.8)

project(AplikacjaSterujaca)
FILE(GLOB_RECURSE head "*.h")
FILE(GLOB_RECURSE sour "*.cpp")
add_custom_target(headers SOURCES  ${head})
add_custom_target(sour SOURCES  ${sour})
add_executable(${PROJECT_NAME} "main.cpp")


