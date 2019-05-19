find_package(Eigen3 REQUIRED)

if (${EIGEN_FOUND})
    include_directories(${EIGEN_INCLUDE_DIR})
endif()