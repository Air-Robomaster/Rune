cmake_minimum_required (VERSION 3.1)
project (RM_BUFF)

message(STATUS "This is Robomaster Vision program:")

# 编译设置
#set(CMAKE_BUILD_TYPE "RELEASE")
#set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread -g")

#设置opencv目录
message(STATUS "operation system is ${CMAKE_SYSTEM}")
if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS "current platform: Windows")
    set(OpenCV_INCLUDE_DIRS C:/OpenCV-3.4.6/include C:/OpenCV-3.4.6/include/opencv C:/OpenCV-3.4.6/include/opencv2)
    set(OpenCV_LIBRARIES C:/OpenCV-3.4.6/x64/mingw/lib/libopencv_img_hash346.dll.a C:/OpenCV-3.4.6/x64/mingw/lib/libopencv_world346.dll.a)
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "    lib path: ${OpenCV_LIBRARIES}")
endif(CMAKE_SYSTEM_NAME MATCHES "Windows")

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    message(STATUS "current platform: Linux ")
    find_package(OpenCV REQUIRED)
    message(STATUS "====================================================")
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "    lib path: ${OpenCV_LIBRARIES}")
endif(CMAKE_SYSTEM_NAME MATCHES "Linux")

# 设置cmake的各种目录
set(CURRENT_PROJET_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CURRENT_PROJET_DIR}/build/bin)
set(LIBRARY_OUTPUT_PATH ${CURRENT_PROJET_DIR}/build/lib)



#设置头文件
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# 添加子目录
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/rune)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/common)
