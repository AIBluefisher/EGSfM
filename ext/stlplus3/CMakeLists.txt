set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(targetname retina_stlplus)

set(headers file_system.hpp portability_fixes.hpp wildcard.hpp)
set(sources file_system.cpp portability_fixes.cpp wildcard.cpp)

include(${PROJECT_SOURCE_DIR}/cmake/SetLibsTargetname.cmake)