set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(targetname solvers)

set(headers estimator.h
            exhaustive_ransac.h
            exhaustive_sampler.h
            inlier_support.h
            lmed_quality_measurement.h
            lmed.h
            mle_quality_measurement.h
            prosac_sampler.h
            prosac.h
            quality_measurement.h
            random_sampler.h
            ransac.h
            sample_consensus_estimator.h
            sampler.h)
set(sources exhaustive_sampler.cpp
            prosac_sampler.cpp
            random_sampler.cpp)

            ####################
## Set   Library  ##
####################
include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)

####################
## Link Libraries ##
####################
target_link_libraries(${targetname} ${GLOG_LIBRARY} 
                                    ${GTEST_LIBRARIES} 
                                    util)

###############
## Unit Test ##
###############
retina_add_test(exhaustive_ransac_test exhaustive_ransac_test.cpp ${targetname})
retina_add_test(exhaustive_sampler_test exhaustive_sampler_test.cpp ${targetname})
retina_add_test(lmed_test lmed_test.cpp ${targetname})
retina_add_test(prosac_test prosac_test.cpp ${targetname})
retina_add_test(random_sampler_test random_sampler_test.cpp ${targetname})
retina_add_test(ransac_test ransac_test.cpp ${targetname})