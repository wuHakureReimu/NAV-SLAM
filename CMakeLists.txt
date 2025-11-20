cmake_minimum_required(VERSION 3.10)

project(NAV-SLAM C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

# 查找jansson库
find_package(PkgConfig REQUIRED)
pkg_check_modules(JANSSON REQUIRED jansson)
if(NOT JANSSON_FOUND)
    message(FATAL_ERROR "未找到 jansson 库！请安装：\nDebian/Ubuntu: sudo apt install libjansson-dev\nmacOS: brew install jansson\nCentOS/RHEL: yum install jansson-devel")
endif()

# -------------------------------
# 添加 pointcloud 库
add_library(pointcloud STATIC
    utils/pointcloud.h
    utils/pointcloud.c
)
target_include_directories(pointcloud
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/utils
)

# -------------------------------
# 添加 kdtree 静态库
add_library(kdtree STATIC
    utils/kdtree.c
    utils/kdtree.h
)
target_include_directories(kdtree
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/utils
)

# kdtree 依赖 pointcloud
target_link_libraries(kdtree
    PUBLIC
        pointcloud
)

# -------------------------------
# main文件
add_executable(mapping_exe
    src/mapping.c
)

target_include_directories(mapping_exe
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/utils
        ${JANSSON_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(mapping_exe
    PRIVATE
        kdtree
        pointcloud
        ${JANSSON_LIBRARIES}
        m
)
