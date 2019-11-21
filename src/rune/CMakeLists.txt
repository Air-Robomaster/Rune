project(rune)
message(STATUS "Configuring sub_project: Rune")

#头文件位置
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJET_DIR}/include/rune
    ${CURRENT_PROJET_DIR}/include/header
    ${CURRENT_PROJET_DIR}/include
)

#添加一个库
add_library(
    rune
    Rune.cpp
    AngleSolver.cpp
)

#添加动态链接库的位置
target_link_libraries(
    rune
    ${OpenCV_LIBRARIES}
)
