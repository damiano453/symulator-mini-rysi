cmake_minimum_required(VERSION 2.8)

add_library("MATH" STATIC "math.c")

project(c)
add_executable(${PROJECT_NAME} "main.c" "extApiPlatform.c" "extApi.c")
target_link_libraries(c "MATH")
