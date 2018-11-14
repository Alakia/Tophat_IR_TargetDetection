CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -march=native -std=c++11")

FIND_PACKAGE(OpenCV REQUIRED)

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/build")

include_directories(./
        )

ADD_EXECUTABLE(TopHat_IR_Detection
        imgprocess.cpp
        )

target_link_libraries(TopHat_IR_Detection
        ${OpenCV_LIBS}
        )