cmake_minimum_required(VERSION 2.8.12)
project (Projection)

find_package (Eigen3 3.4 REQUIRED)
find_package(OpenCV REQUIRED)

set (CMAKE_SHARED_LINKER_FLAGS "-pthread")

add_library(Projection STATIC ${CMAKE_SOURCE_DIR}/src/Projection.cpp)

target_link_libraries(Projection PUBLIC Eigen3::Eigen)
target_include_directories(Projection PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(Projection PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(Projection PUBLIC ${OpenCV_LIBS})

add_executable(example_3d_projection ${CMAKE_SOURCE_DIR}/example_3d_projection.cpp)

target_include_directories(example_3d_projection PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(example_3d_projection PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(example_3d_projection PUBLIC Projection)
target_link_libraries(example_3d_projection PUBLIC Eigen3::Eigen)
target_link_libraries(example_3d_projection PUBLIC ${OpenCV_LIBS})