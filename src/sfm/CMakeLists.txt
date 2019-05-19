set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(targetname sfm)

set(headers sfm_reconstructor.h)
set(sources sfm_reconstructor.cpp)

include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)
target_link_libraries(sfm geometry 
                          util 
                          vot_utils
                          vot_vocabtree
                          ${GLOG_LIBRARY}
                          ${OPENMVG_LIBRARIES})