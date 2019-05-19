set(targetname log)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(headers rang.hpp log.h)
set(sources log.cpp)

include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)