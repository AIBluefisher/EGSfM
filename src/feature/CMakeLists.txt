set(targetname feature)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(headers sift.h 
            feature.h 
            fisher_vector_extractor.h 
            global_descriptor_extractor.h)
set(sources sift.cpp 
            feature.cpp 
            fisher_vector_extractor.cpp)

# include(${PROJECT_SOURCE_DIR}/cmake/FindOpenImageIO.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/FindVlFeat.cmake)

# set target name
include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)

# include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenImageIO.cmake)

target_link_libraries(${targetname} 
                      retina_vlfeat
                    #   image
                      util)  