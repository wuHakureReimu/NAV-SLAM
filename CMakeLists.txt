cmake_minimum_required(VERSION 3.10)

project(NAV-SLAM C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 设置编译器选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")

# 包含项目头文件目录
include_directories(headers utils)

# 查找数学库（通常不需要显式查找，但这里显式声明）
find_package(Threads REQUIRED)

# 查找jansson库
find_package(PkgConfig REQUIRED)
pkg_check_modules(JANSSON REQUIRED jansson)


# 添加可执行文件
add_executable(main.exe
    src/main.c
    src/ekf.c
    src/slam.c
    utils/kdtree.c
    utils/matrix.c
    utils/pointcloud.c
)

# 添加包含目录
target_include_directories(main.exe PRIVATE 
    headers
    utils
    ${JANSSON_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(main.exe PRIVATE
    m  # 数学库
    ${JANSSON_LIBRARIES}
)
