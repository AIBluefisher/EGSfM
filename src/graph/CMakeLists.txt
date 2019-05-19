set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(targetname graph)

set(headers graph.h 
            graph_cut.h
            union_find.h
            viewing_graph.h)
set(sources graph_cut.cpp
            union_find.cpp
            viewing_graph.cpp)

####################
## Find Libraries ##
####################
include(${PROJECT_SOURCE_DIR}/cmake/FindSTLPlus3.cmake)

####################
## Set   Library  ##
####################
include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)

####################
## Link Libraries ##
####################
include(${PROJECT_SOURCE_DIR}/cmake/LinkSTLPlus3.cmake)
target_link_libraries(${targetname} ${GLOG_LIBRARY} 
                                    graclus
                                    log 
                                    matching)


###############
## Unit Test ##
###############
retina_add_test(union_find_test union_find_test.cpp ${targetname})
retina_add_test(graph_test graph_test.cpp ${targetname})
retina_add_test(graph_cut_test graph_cut_test.cpp graclus ${GLOG_LIBRARY} ${GTEST_LIBRARIES} ${targetname})
retina_add_test(viewing_graph_test viewing_graph_test.cpp ${targetname})