set(targetname matching)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(headers matcher.h)
set(sources matcher.cpp)

include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)

target_link_libraries(${targetname} ${OPENMVG_LIBRARIES})