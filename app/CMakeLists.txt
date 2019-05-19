# executables
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(${PROJECT_SOURCE_DIR}/cmake/FindSTLPlus3.cmake)


add_executable(vg_matching vg_matching.cpp)
target_link_libraries(vg_matching graph 
                                  gomp
                                  ${GLOG_LIBRARY}
                                  ${GFLAGS_LIBRARIES}
                                  ${OPENMVG_LIBRARIES})

add_executable(fisher_vector_similarity fisher_vector_similarity.cpp)
target_link_libraries(fisher_vector_similarity feature 
                                               ${GLOG_LIBRARY} 
                                               ${GFLAGS_LIBRARIES}
                                               retina_stlplus)

add_executable(fisher_vector_trainer fisher_vector_trainer.cpp)
target_link_libraries(fisher_vector_trainer feature 
                                            ${GLOG_LIBRARY} 
                                            ${GFLAGS_LIBRARIES}
                                            retina_stlplus)

add_executable(image_batch_rename image_batch_rename.cpp)
target_link_libraries(image_batch_rename retina_stlplus)

add_executable(image_cluster image_cluster.cpp)
target_link_libraries(image_cluster geometry 
                                    ${OPENMVG_LIBRARIES} 
                                    ${GLOG_LIBRARY}
                                    ${GFLAGS_LIBRARIES}
                                    retina_stlplus)

add_executable(sfm_align sfm_align.cpp)
target_link_libraries(sfm_align geometry
                                ${OPENMVG_LIBRARIES}
                                ${GLOG_LIBRARY}
                                ${GFLAGS_LIBRARIES})

add_executable(sfm_reconstruction sfm_reconstruction.cpp)
target_link_libraries(sfm_reconstruction sfm
                                        #  ${OPENMVG_LIBRARIES}
                                         ${GLOG_LIBRARY}
                                         ${GFLAGS_LIBRARIES}
                                         yaml-cpp)