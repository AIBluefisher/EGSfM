set(targetname math)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(headers distribution.h
            l1_solver.h
            rotation.h
            sparse_cholesky_llt.h
            util.h)

set(sources rotation.cpp
            sparse_cholesky_llt.cpp)

include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)
target_link_libraries(math util ${GLOG_LIBRARY})