set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(headers aib.h
            array.h
            covdet.h
            dsift.h
            fisher.h
            float.th
            generic.h
            getopt_long.h
            gmm.h
            heap-def.h
            hikmeans.h
            hog.h
            homkermap.h
            host.h
            ikmeans.h
            # imopv_sse2.h
            imopv.h
            kdtree.h
            kmeans.h
            lbp.h
            liop.h
            mathop_avx.h
            # mathop_sse2.h
            mathop.h
            mser.h
            pgm.h
            qsort-def.h
            quickshift.h
            random.h
            rodrigues.h
            scalespace.h
            shuffle-def.h 
            sift.h 
            slic.h 
            stringop.h
            svm.h 
            svmdataset.h 
            vlad.h)       

            
set(sources array.c
            covdet.c
            dsift.c
            fisher.c
            generic.c
            getopt_long.c
            gmm.c
            hikmeans.c
            hog.c
            homkermap.c
            host.c
            ikmeans_elkan.tc
            ikmeans_init.tc
            ikmeans_lloyd.tc
            # imopv_sse2.c
            imopv.c
            kdtree.c
            kmeans.c
            lbp.c
            liop.c
            mathop_avx.c
            # mathop_sse2.c
            mathop.c
            mser.c
            pgm.c
            quickshift.c
            random.c
            rodrigues.c
            scalespace.c
            sift.c 
            slic.c 
            stringop.c
            svm.c 
            svmdataset.c 
            vlad.c)

# followed with (https://github.com/colmap/colmap/blob/dev/lib/VLFeat/CMakeLists.txt)
if(SIMD_ENABLED AND IS_X86)
    if(MSVC)
        add_definitions("-DVL_DISABLE_AVX")
    else()
        set(AVX_SOURCES mathop_avx.c mathop_avx.h)
    endif()

    set(SSE2_SOURCES imopv_sse2.c
                     imopv_sse2.h
                     mathop_sse2.c
                     mathop_sse2.h)
    
    list(APPEND VLFEAT_SOURCE_FILES ${AVX_SOURCES} ${SSE2_SOURCES})

    if(MSVC)
        set_source_files_properties(${AVX_SOURCES}
            PROPERTIES COMPILE_FLAGS "/arch:AVX")
        set_source_files_properties(${SSE2_SOURCES}
            PROPERTIES COMPILE_FLAGS "arch:SSE2 /D__SSE2__")
    else()
        set_source_files_properties(${AVX_SOURCES}
            PROPERTIES COMPILE_FLAGS "-mavx")
        set_source_files_properties(${SSE2_SOURCES}
            PROPERTIES COMPILE_FLAGS "-msse2")
    endif()
else()
    add_definitions("-DVL_DISABLE_AVX")
    add_definitions("-DVL_DISABLE_SSE2")
endif()

if(NOT OPENMP_ENABLED OR NOT OPENMP_FOUND)
    add_definitions("-DVL_DISABLE_OPENMP")
endif()

add_library(retina_vlfeat ${headers} ${sources})
# https://stackoverflow.com/questions/25617839/undefined-reference-to-symbol-pthread-key-deleteglibc-2-2-5
target_link_libraries(retina_vlfeat pthread)