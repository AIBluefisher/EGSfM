set(targetname util)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(headers hash.h
            map_util.h
            ply_helper.h
            random.h
            stringprintf.h
            types.h 
            util.h
            )

set(sources stringprintf.cpp
            random.cpp
            )

include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)