#工作目录项目名称
project(common)
message(STATUS "Configuring sub_project: common")

#包括的头文件linux
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJET_DIR}/include/rune
    ${CURRENT_PROJET_DIR}/include/header
    ${CURRENT_PROJET_DIR}/include
)

#获取文件路径的所有源文件
#add_sourcr_directory(. DIR_SRCS)

#输出的.cpp的文件
add_executable(main main.cpp)

#项目包括的目录
target_include_directories(main
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CURRENT_PROJET_DIR}/include
)
#链接时用，指明链接的路径
#link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                 # ${EXECUTABLE_OUTPUT_PATH})

target_link_libraries(
    main
    ${OpenCV_LIBRARIES}
    rune
)