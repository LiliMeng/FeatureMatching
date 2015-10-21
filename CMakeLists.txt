cmake_minimum_required(VERSION 2.0)
project(VisualSLAM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")
find_package( OpenCV REQUIRED )
set(SOURCE_FILES
    ReadRGBD.h
    ReadRGBD.cpp
    Features.h
    Features.cpp
    VisualSLAM.h
    VisualSLAM.cpp    
    main.cpp)

add_executable(VisualSLAM ${SOURCE_FILES})
target_link_libraries( VisualSLAM ${OpenCV_LIBS} )
