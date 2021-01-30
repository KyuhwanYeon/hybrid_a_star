cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_CXX_STANDARD 11)

project(HYBRID_A_STAR)
include_directories(include)
set(sources src/main.cpp src/hybrid-a-star.cpp src/viewer.cpp)
find_package(OpenCV REQUIRED)
add_executable(HYBRID_A_STAR ${sources})
target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
)

