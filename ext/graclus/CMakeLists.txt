set(CMAKE_CXX_STANDARD 11)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
    .
)
file(GLOB GRACLUS_HEADER_FILES "metisLib/*.h" "graclus/*.h")
file(GLOB GRACLUS_SOURCE_FILES "metisLib/*.c" "graclus/*.c")

add_library(graclus
    ${GRACLUS_HEADER_FILES}
    ${GRACLUS_SOURCE_FILES}
    multilevelLib/wkkm.c
    multilevelLib/mlkkm.c
)