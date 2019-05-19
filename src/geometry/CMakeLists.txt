set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(targetname geometry)

# include_directories(${PROJECT_SOURCE_DIR}/src/geometry)

set(headers bundle_adjustment.h
            image_cluster.h
            ransac_similarity.h
            ransac.h
            sfm_aligner.h
            sfm_optimizer.h
            sim3.h
            similarity_graph_optimization.h
            )
set(sources bundle_adjustment.cpp
            image_cluster.cpp
            sfm_aligner.cpp
            sfm_optimizer.cpp
            similarity_graph_optimization.cpp
            )

####################
## Set   Library  ##
####################
include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)

target_link_libraries(${targetname} ${OPENMVG_LIBRARIES} 
                                    ${GLOG_LIBRARY}
                                    graclus 
                                    log 
                                    graph 
                                    util
                                    math
                                    solvers)