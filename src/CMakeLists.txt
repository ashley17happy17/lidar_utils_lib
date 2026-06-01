cmake_minimum_required(VERSION 3.14)
project(PointCloudUtils VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 尋找 PCL 與 Eigen3 (這會直接抓取外部主專案目前正在使用的版本)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features search kdtree)
find_package(Eigen3 REQUIRED)

# 蒐集所有實作原始碼
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

# 建立靜態程式庫
add_library(lidar_utils STATIC ${SRC_FILES})

# 設定頭文件路徑：將 include 設為 PUBLIC 分享給外部，src 設為 PRIVATE 鎖在內部
target_include_directories(lidar_utils 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${PCL_INCLUDE_DIRS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(lidar_utils PUBLIC 
    ${PCL_LIBRARIES} 
    Eigen3::Eigen
)
