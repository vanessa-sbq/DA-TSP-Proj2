cmake_minimum_required(VERSION 3.26)
project(DA_TSP_Proj2)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_executable(DA_TSP_Proj2
        src/Application.cpp
        src/Application.h
        src/GeoPoint.cpp
        src/GeoPoint.h
        src/Graph.h
        src/main.cpp
        src/TSP.cpp
        src/TSP.h)
