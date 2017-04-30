cmake_minimum_required(VERSION 2.8)

project(AplikacjaSterujaca)
FILE(GLOB_RECURSE head "*.h")

add_custom_target(headers SOURCES  ${head})
add_custom_target(sour SOURCES  ${sour})
add_executable(${PROJECT_NAME} "main.cpp" "robotsimswarmapibody.cpp" "robot.cpp")



