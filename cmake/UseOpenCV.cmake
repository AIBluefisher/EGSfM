option(WITH_OPENCV "Build with OpenCV support?" OFF)

if(WITH_OPENCV)
    find_package(OpenCV REQUIRED)

    if(${OpenCV_FOUND})
        message(STATUS "OpenCV found in " ${OpenCV_INCLUDE_DIRS})
        include_directories(include ${OpenCV_INCLUDE_DIRS})
        add_definitions(-DUSE_OPENCV)
    else()
        message(STATUS "OpenCV required but not found")
    endif()
endif()