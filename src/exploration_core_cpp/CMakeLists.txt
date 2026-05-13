cmake_minimum_required(VERSION 3.14)
project(rbcv_plan_tour CXX)

set(CMAKE_CXX_STANDARD 17)

add_executable(rbcv_plan_tour src/rbcv_plan_tour.cpp)

if(MSVC)
  target_compile_options(rbcv_plan_tour PRIVATE /W4)
else()
  target_compile_options(rbcv_plan_tour PRIVATE -Wall -Wextra -O3)
endif()
