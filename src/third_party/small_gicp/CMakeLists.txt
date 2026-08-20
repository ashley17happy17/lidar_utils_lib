# small_gicp — vendored dependency, see README.upstream.md for provenance.
#
# 只建置 upstream 的 "helper library"（registration.cpp / registration_helper.cpp），
# 不使用 upstream 的 CMakeLists.txt，以避免它連帶要求 fmt / TBB / PCL /
# Iridescence / pybind11 / GTest，也避免它的 cache 變數污染母專案。

# 本函式庫常以 add_subdirectory(third_party/lidar_utils_lib/src) 的方式被上層專案引用，
# 而上層專案可能已經自備 small_gicp（例如 lidar_slam 自己 FetchContent 一份）。
# 若目標已存在就沿用上層的版本，否則會發生 "target already exists" 設定錯誤。
if(TARGET small_gicp)
  message(STATUS "small_gicp: 使用上層專案已提供的 target，跳過 vendored 版本")
else()
  add_library(small_gicp STATIC
    src/small_gicp/registration/registration.cpp
    src/small_gicp/registration/registration_helper.cpp
  )

  target_include_directories(small_gicp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(small_gicp PUBLIC
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
  )

  target_compile_features(small_gicp PUBLIC cxx_std_17)

  # lidar_utils 可能被連進 shared library，靜態相依必須是 PIC
  set_target_properties(small_gicp PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
