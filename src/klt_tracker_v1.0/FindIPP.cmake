cmake_policy(PUSH)
cmake_minimum_required(VERSION 2.6)
cmake_policy(POP)

#IF( NOT IPP_FOUND )

FILE(GLOB IPP_INCLUDE_PATHS_1 "/opt/intel/composer_xe_2013.1.117/ipp/include")

SET(
    IPP_INCLUDE_PATHS
    ${IPP_INCLUDE_PATHS_1}
)


SET(IPP_INCLUDE_DIR ${IPP_INCLUDE_PATHS_1})

SET(IPPROOT "${IPP_INCLUDE_DIR}/..")
SET(IPP_PATH "${IPP_INCLUDE_DIR}/../lib/intel64/")


            file(GLOB IPP_HDRS "${IPP_PATH}/../../include")
                set(IPP_FOUND TRUE)

    add_definitions(-DHAVE_IPP)
    include_directories(${IPP_INCLUDE_DIR})
link_directories(${IPP_PATH})
link_directories("/opt/intel/composer_xe_2013.1.117/compiler/lib/intel64")
    file(GLOB em64t_files "${IPP_PATH}/../../lib/*em64t*")
    set(IPP_ARCH)
    if(em64t_files)
        set(IPP_ARCH "em64t")
    endif()

    set(A ${CMAKE_STATIC_LIBRARY_PREFIX})
    #set(B ${IPP_ARCH}${CMAKE_STATIC_LIBRARY_SUFFIX})
    set(B ${IPP_ARCH}${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(L)
    set(IPP_LIBS ${A}ippcore${B} ${A}ipps${B}  ${A}ippi${B})
#                 ${A}svml${B} ${A}imf${B} ${A}iomp5${B}   ${A}irc${B} ${A}pthread${B})

#message(STATUS "IPPPATH = "  ${IPP_PATH})
#message(STATUS "IPP_LIBS ="  ${IPP_LIBS})

#ENDIF( NOT IPP_FOUND )
