set(GTEST_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ext/googletest/include)
set(GTEST_LIBRARIES gtest gtest_main)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/test)

include_directories(${GTEST_INCLUDE_DIR})

macro(retina_add_test TARGET_TEST_NAME TARGET_TEST_SOURCES)
    add_executable(${TARGET_TEST_NAME} ${TARGET_TEST_SOURCES})
    target_link_libraries(${TARGET_TEST_NAME} ${GTEST_LIBRARIES} ${ARGN})
    add_test(${TARGET_TEST_NAME} ${EXECUTABLE_OUTPUT_PATH}/${TARGET_TEST_NAME})
endmacro(retina_add_test)