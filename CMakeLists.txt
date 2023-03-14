cmake_minimum_required (VERSION 3.13)
project (GAMES101)

set(BUILD_TESTING OFF)
add_subdirectory(eigen)

find_package(Eigen3 REQUIRED)

add_subdirectory(lab0)
target_link_libraries(Transformation PRIVATE Eigen3::Eigen)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/opencv/build")
find_package(OpenCV REQUIRED)


add_subdirectory(lab3)
message(STATUS "${OpenCV_LIB_PATH}")
message(STATUS "${OpenCV_LIBS}")
target_link_libraries(Rasterizer PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
target_link_directories(Rasterizer PRIVATE  ${OpenCV_LIB_PATH})

add_subdirectory(lab4)
target_link_libraries(BezierCurve PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
target_link_directories(BezierCurve PRIVATE  ${OpenCV_LIB_PATH})

add_subdirectory(lab6)
target_compile_options(RayTracing PUBLIC -Wall  -pedantic -fsanitize=undefined)
target_compile_features(RayTracing PUBLIC cxx_std_17)
target_link_libraries(RayTracing PUBLIC -fsanitize=undefined)
