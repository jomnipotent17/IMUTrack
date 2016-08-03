cmake_policy(PUSH)
cmake_minimum_required(VERSION 2.6)
cmake_policy(POP)

#IF( NOT IPP_FOUND )

FILE(GLOB IPP_INCLUDE_PATHS_0 "$ENV{ProgramFiles}/Intel/IPP/*.*/ia32/include")
FILE(GLOB IPP_INCLUDE_PATHS_1 "/opt/intel/ipp/*.*/ia32/include")
FILE(GLOB IPP_INCLUDE_PATHS_2 "/usr/local/intel/ipp/*.*/ia32/include")

SET(
    IPP_INCLUDE_PATHS
    ${IPP_INCLUDE_PATHS_0}
    ${IPP_INCLUDE_PATHS_1}
    ${IPP_INCLUDE_PATHS_2}
)

FIND_PATH(
  IPP_INCLUDE_DIR
  ippi.h
  ${IPP_INCLUDE_PATHS}
)

SET(IPPROOT "${IPP_INCLUDE_DIR}/..")

        if(WIN32)
            find_path(IPP_PATH "ippi.dll"
                PATHS ${CMAKE_PROGRAM_PATH} ${CMAKE_SYSTEM_PROGRAM_PATH} ${IPPROOT}
                PATH_SUFFIXES bin
                DOC "The path to IPP dynamic libraries")
            if(NOT IPP_PATH)
                find_path(IPP_PATH "ippiem64t-${v}.dll"
                    PATHS ${CMAKE_PROGRAM_PATH} ${CMAKE_SYSTEM_PROGRAM_PATH}
                    PATH_SUFFIXES bin
                    DOC "The path to IPP dynamic libraries")
            endif()
        endif()
        if(UNIX)
            find_path(IPP_PATH "libippi${CMAKE_SHARED_LIBRARY_SUFFIX}"
                PATHS ${CMAKE_LIBRARY_PATH} ${CMAKE_SYSTEM_LIBRARY_PATH} ${IPPROOT}
                PATH_SUFFIXES lib studlib sharedlib
                DOC "The path to IPP dynamic libraries")
            if(NOT IPP_PATH)
                find_path(IPP_PATH "libippiem64t${CMAKE_SHARED_LIBRARY_SUFFIX}"
                    PATHS ${CMAKE_LIBRARY_PATH} ${CMAKE_SYSTEM_LIBRARY_PATH} ${IPPROOT}
                    PATH_SUFFIXES lib studlib sharedlib
                    DOC "The path to IPP dynamic libraries")
            endif()
        endif()
        if(IPP_PATH)
            file(GLOB IPP_HDRS "${IPP_PATH}/../include")
            if(IPP_HDRS)
                set(IPP_FOUND TRUE)
            endif()
        endif()

    add_definitions(-DHAVE_IPP)
    include_directories("${IPP_PATH}/../include")
    link_directories("${IPP_PATH}/../sharedlib")

    file(GLOB em64t_files "${IPP_PATH}/../lib/*em64t*")
    set(IPP_ARCH)
    if(em64t_files)
        set(IPP_ARCH "em64t")
    endif()

    set(A ${CMAKE_STATIC_LIBRARY_PREFIX})
    #set(B ${IPP_ARCH}${CMAKE_STATIC_LIBRARY_SUFFIX})
    set(B ${IPP_ARCH}${CMAKE_SHARED_LIBRARY_SUFFIX})
    if(WIN32)
        set(L l)
    else()
        set(L)
    endif()
    set(IPP_LIBS ${A}ippcore${B} ${A}ipps${B}  ${A}ippi${B})
#                 ${A}svml${B} ${A}imf${B} ${A}iomp5${B}   ${A}irc${B} ${A}pthread${B})

#message(STATUS "IPPPATH = "  ${IPP_PATH})
#message(STATUS "IPP_LIBS ="  ${IPP_LIBS})

#ENDIF( NOT IPP_FOUND )
