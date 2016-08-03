# - Try to find OpenCV library installation
# See http://sourceforge.net/projects/opencvlibrary/
#
# The follwoing variables are optionally searched for defaults
#  OpenCV_ROOT_DIR:            Base directory of OpenCv tree to use.
#  OpenCV_FIND_REQUIRED_COMPONENTS : FIND_PACKAGE(OpenCV COMPONENTS ..) 
#    compatible interface. typically  CV CXCORE CVAUX HIGHGUI CVCAM .. etc.
#
# The following are set after configuration is done: 
#  OpenCV_FOUND
#  OpenCV_INCLUDE_DIR
#  OpenCV_LIBRARIES
#  OpenCV_LINK_DIRECTORIES
#
# deprecated:
#  OPENCV_* uppercase replaced by case sensitive OpenCV_*
#  OPENCV_EXE_LINKER_FLAGS
#  OPENCV_INCLUDE_DIR : replaced by plural *_DIRS
# 
# 2004/05 Jan Woetzel, Friso, Daniel Grest 
# 2006/01 complete rewrite by Jan Woetzel
# 1006/09 2nd rewrite introducing ROOT_DIR and PATH_SUFFIXES 
#   to handle multiple installed versions gracefully by Jan Woetzel
#
# tested with:
# -OpenCV 0.97 (beta5a):  MSVS 7.1, gcc 3.3, gcc 4.1
# -OpenCV 0.99 (1.0rc1):  MSVS 7.1, MSVS 8.0
#
# 20 Nov 2008: Changes by Jose Luis Blanco Claraco <http://www.isa.uma.es/jlblanco>
#    - Updated for finding OpenCV1.1.0-pre under Windows in the system 
#        registry (It can be installed along 1.0.0 without problems)
#    - Finds and uses debug libraries and DLLs (Windows).
#
# www.mip.informatik.uni-kiel.de/~jw
# --------------------------------

# NOTES
#  NO_CMAKE_SYSTEM_PATH is required to bypass searching in CMAKE_SYSTEM_LIBRARY_PATH
#  because it defaults to $ENV{ProgramFiles} in WindowsPaths.cmake 
#  which corrupts the order of BIAS_EXTERN_LIBS  (JW 02/2007)


MACRO(DBG_MSG _MSG)
  #MESSAGE(STATUS "${CMAKE_CURRENT_LIST_FILE}(${CMAKE_CURRENT_LIST_LINE}):\n${_MSG}")
ENDMACRO(DBG_MSG)



# required cv components with header and library if COMPONENTS unspecified
IF    (NOT OpenCV_FIND_COMPONENTS)
  # default
  SET(OpenCV_FIND_REQUIRED_COMPONENTS   CV CXCORE CVAUX HIGHGUI )
  IF   (WIN32)
#    LIST(APPEND OpenCV_FIND_REQUIRED_COMPONENTS  CVCAM ) # WIN32 only actually
  ENDIF(WIN32)  
ENDIF (NOT OpenCV_FIND_COMPONENTS)


# typical root dirs of installations, exactly one of them is used
FILE(GLOB OPENCV2_PATHS "$ENV{ProgramFiles}/../OpenCV*.*")

# typical root dirs of installations, exactly one of them is used
SET (OpenCV_POSSIBLE_ROOT_DIRS
  "${OpenCV_ROOT_DIR}"
  "$ENV{OpenCV_ROOT_DIR}"  
  "$ENV{OPENCV_DIR}"  # only for backward compatibility deprecated by ROOT_DIR
  "$ENV{OPENCV_HOME}" # only for backward compatibility
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]"
  "$ENV{ProgramFiles}/OpenCV"
  "${OPENCV2_PATHS}"
  /usr/local
  /usr
  )


# MIP Uni Kiel /opt/net network installation 
# get correct prefix for current gcc compiler version for gcc 3.x  4.x
IF    (${CMAKE_COMPILER_IS_GNUCXX})
  IF    (NOT OpenCV_FIND_QUIETLY)
    MESSAGE(STATUS "Checking GNUCXX version 3/4 to determine  OpenCV /opt/net/ path")
  ENDIF (NOT OpenCV_FIND_QUIETLY)
  EXEC_PROGRAM(${CMAKE_CXX_COMPILER} ARGS --version OUTPUT_VARIABLE CXX_COMPILER_VERSION)  
  IF   (CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")
    SET(IS_GNUCXX3 TRUE)
    LIST(APPEND OpenCV_POSSIBLE_ROOT_DIRS /opt/net/gcc33/OpenCV )
  ENDIF(CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")  
  IF   (CXX_COMPILER_VERSION MATCHES ".*4\\.[0-1].*")
    SET(IS_GNUCXX4 TRUE)
    LIST(APPEND OpenCV_POSSIBLE_ROOT_DIRS /opt/net/gcc41/OpenCV )
  ENDIF(CXX_COMPILER_VERSION MATCHES ".*4\\.[0-1].*")
  #SuSE 10.3, gcc4.2.1 needs OpenCV-1.0.0
  IF   (CXX_COMPILER_VERSION MATCHES ".*4\\.[2-9].*")
    SET(IS_GNUCXX4 TRUE)
    LIST(APPEND OpenCV_POSSIBLE_ROOT_DIRS /opt/net/gcc41/OpenCV-1.0.0)
  ENDIF(CXX_COMPILER_VERSION MATCHES ".*4\\.[2-9].*")
ENDIF (${CMAKE_COMPILER_IS_GNUCXX})

#DBG_MSG("OpenCV_POSSIBLE_ROOT_DIRS=${OpenCV_POSSIBLE_ROOT_DIRS}")

#
# select exactly ONE OpenCV base directory/tree 
# to avoid mixing different version headers and libs
#
FIND_PATH(OpenCV_ROOT_DIR 
  NAMES 
  cv/include/cv.h     # windows
  include/opencv/cv.h # linux /opt/net
  include/cv/cv.h 
  include/cv.h 
  PATHS ${OpenCV_POSSIBLE_ROOT_DIRS})
DBG_MSG("OpenCV_ROOT_DIR=${OpenCV_ROOT_DIR}")


# header include dir suffixes appended to OpenCV_ROOT_DIR
SET(OpenCV_INCDIR_SUFFIXES
  include
  include/cv
  include/opencv
  cv/include
  cxcore/include
  cvaux/include
  otherlibs/cvcam/include
  otherlibs/highgui
  otherlibs/highgui/include
  otherlibs/_graphics/include
  )
DBG_MSG("OpenCV_INCDIR_SUFFIXES=${OpenCV_INCDIR_SUFFIXES}")

# library linkdir suffixes appended to OpenCV_ROOT_DIR 
SET(OpenCV_LIBDIR_SUFFIXES
  lib
  OpenCV/lib
  otherlibs/_graphics/lib
  )
DBG_MSG("OpenCV_LIBDIR_SUFFIXES=${OpenCV_LIBDIR_SUFFIXES}")


#
# find incdir for each lib
#
FIND_PATH(OpenCV_CV_INCLUDE_DIR
  NAMES cv.h      
  PATHS ${OpenCV_ROOT_DIR} 
  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )
FIND_PATH(OpenCV_CXCORE_INCLUDE_DIR   
  NAMES cxcore.h
  PATHS ${OpenCV_ROOT_DIR} 
  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )
FIND_PATH(OpenCV_CVAUX_INCLUDE_DIR    
  NAMES cvaux.h
  PATHS ${OpenCV_ROOT_DIR} 
  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )
FIND_PATH(OpenCV_HIGHGUI_INCLUDE_DIR  
  NAMES highgui.h 
  PATHS ${OpenCV_ROOT_DIR} 
  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )
#FIND_PATH(OpenCV_CVCAM_INCLUDE_DIR    
#  NAMES cvcam.h 
#  PATHS ${OpenCV_ROOT_DIR} 
#  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )

#
# find sbsolute path to all libraries 
# some are optionally, some may not exist on Linux
#
FIND_LIBRARY(OpenCV_CV_LIBRARY   
  NAMES cv opencv cv200 cv210
  PATHS ${OpenCV_ROOT_DIR}  
  PATH_SUFFIXES  ${OpenCV_LIBDIR_SUFFIXES} )
FIND_LIBRARY(OpenCV_CVAUX_LIBRARY
  NAMES cvaux cvaux200 cvaux210
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
#FIND_LIBRARY(OpenCV_CVCAM_LIBRARY   
#  NAMES cvcam
#  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} ) 
#FIND_LIBRARY(OpenCV_CVHAARTRAINING_LIBRARY
#  NAMES cvhaartraining
#  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} ) 
FIND_LIBRARY(OpenCV_CXCORE_LIBRARY  
  NAMES cxcore cxcore200 cxcore210
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
#FIND_LIBRARY(OpenCV_CXTS_LIBRARY   
#  NAMES cxts
#  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
FIND_LIBRARY(OpenCV_HIGHGUI_LIBRARY  
  NAMES highgui highgui200 highgui210
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
#FIND_LIBRARY(OpenCV_ML_LIBRARY  
#  NAMES ml
#  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
#FIND_LIBRARY(OpenCV_TRS_LIBRARY  
#  NAMES trs
#  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )

# DEBUG VERSIONS:
FIND_LIBRARY(OpenCV_CVd_LIBRARY   
  NAMES cvd opencvd cv200d cv210d
  PATHS ${OpenCV_ROOT_DIR}  
  PATH_SUFFIXES  ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CVd_LIBRARY=${OpenCV_CVd_LIBRARY}")
FIND_LIBRARY(OpenCV_CVAUXd_LIBRARY
  NAMES cvauxd cvaux200d cvaux210d
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES}
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CVAUXd_LIBRARY=${OpenCV_CVAUXd_LIBRARY}")
FIND_LIBRARY(OpenCV_CXCOREd_LIBRARY  
  NAMES cxcored cxcore200d cxcore210d
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CXCOREd_LIBRARY=${OpenCV_CXCOREd_LIBRARY}")
FIND_LIBRARY(OpenCV_HIGHGUId_LIBRARY  
  NAMES highguid highgui200d highgui210d
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_HIGHGUId_LIBRARY=${OpenCV_HIGHGUId_LIBRARY}")


#
# Logic selecting required libs and headers
#
SET(OpenCV_FOUND ON)
DBG_MSG("OpenCV_FIND_REQUIRED_COMPONENTS=${OpenCV_FIND_REQUIRED_COMPONENTS}")
FOREACH(NAME ${OpenCV_FIND_REQUIRED_COMPONENTS} )

  # only good if header and library both found   
  IF    (OpenCV_${NAME}_INCLUDE_DIR AND OpenCV_${NAME}_LIBRARY)
    LIST(APPEND OpenCV_INCLUDE_DIRS ${OpenCV_${NAME}_INCLUDE_DIR} )
    IF (EXISTS "${OpenCV_${NAME}_LIBRARY}" AND EXISTS "${OpenCV_${NAME}d_LIBRARY}")
        #MESSAGE(STATUS "DEBUG ***************  ${OpenCV_${NAME}d_LIBRARY}")
        LIST(APPEND OpenCV_LIBRARIES    debug;${OpenCV_${NAME}d_LIBRARY};optimized;${OpenCV_${NAME}_LIBRARY} )
    ELSE(EXISTS "${OpenCV_${NAME}_LIBRARY}" AND EXISTS "${OpenCV_${NAME}d_LIBRARY}")
        LIST(APPEND OpenCV_LIBRARIES    ${OpenCV_${NAME}_LIBRARY} )
    ENDIF(EXISTS "${OpenCV_${NAME}_LIBRARY}" AND EXISTS "${OpenCV_${NAME}d_LIBRARY}")
    DBG_MSG("appending for NAME=${NAME} ${OpenCV_${NAME}_INCLUDE_DIR} and ${OpenCV_${NAME}_LIBRARY}" )
  ELSE  (OpenCV_${NAME}_INCLUDE_DIR AND OpenCV_${NAME}_LIBRARY)
    DBG_MSG("OpenCV component NAME=${NAME} not found! "
      "\nOpenCV_${NAME}_INCLUDE_DIR=${OpenCV_${NAME}_INCLUDE_DIR} "
      "\nOpenCV_${NAME}_LIBRARY=${OpenCV_${NAME}_LIBRARY} ")
    SET(OpenCV_FOUND OFF)
  ENDIF (OpenCV_${NAME}_INCLUDE_DIR AND OpenCV_${NAME}_LIBRARY)
  
ENDFOREACH(NAME)

DBG_MSG("OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
DBG_MSG("OpenCV_LIBRARIES=${OpenCV_LIBRARIES}")

# get the link directory for rpath to be used with LINK_DIRECTORIES: 
IF (OpenCV_CV_LIBRARY)
  GET_FILENAME_COMPONENT(OpenCV_LINK_DIRECTORIES ${OpenCV_CV_LIBRARY} PATH)
ENDIF (OpenCV_CV_LIBRARY)

MARK_AS_ADVANCED(
  OpenCV_ROOT_DIR
  OpenCV_INCLUDE_DIRS
  OpenCV_CV_INCLUDE_DIR
  OpenCV_CXCORE_INCLUDE_DIR
  OpenCV_CVAUX_INCLUDE_DIR
  OpenCV_CVCAM_INCLUDE_DIR
  OpenCV_HIGHGUI_INCLUDE_DIR
  OpenCV_LIBRARIES
  OpenCV_CV_LIBRARY
  OpenCV_CXCORE_LIBRARY
  OpenCV_CVAUX_LIBRARY
  OpenCV_CVCAM_LIBRARY
  OpenCV_CVHAARTRAINING_LIBRARY
  OpenCV_CXTS_LIBRARY
  OpenCV_HIGHGUI_LIBRARY
  OpenCV_ML_LIBRARY
  OpenCV_TRS_LIBRARY
  OpenCV_CVd_LIBRARY
  OpenCV_CXCOREd_LIBRARY
  OpenCV_CVAUXd_LIBRARY
  OpenCV_CVCAMd_LIBRARY
  OpenCV_CVHAARTRAININGd_LIBRARY
  OpenCV_CXTSd_LIBRARY
  OpenCV_HIGHGUId_LIBRARY
  OpenCV_MLd_LIBRARY
  OpenCV_TRSd_LIBRARY
  )


# be backward compatible:
SET(OPENCV_LIBRARIES   ${OpenCV_LIBRARIES} )
SET(OPENCV_INCLUDE_DIR ${OpenCV_INCLUDE_DIRS} )
SET(OPENCV_FOUND       ${OpenCV_FOUND})
SET(OpenCV_LIBS        ${OpenCV_LIBRARIES} )



# display help message
IF(NOT OpenCV_FOUND)
  # make FIND_PACKAGE friendly
  IF(NOT OpenCV_FIND_QUIETLY)
    IF(OpenCV_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR
        "OpenCV required but some headers or libs not found. Please specify it's location with OpenCV_ROOT_DIR env. variable.")
    ELSE(OpenCV_FIND_REQUIRED)
      MESSAGE(STATUS 
        "ERROR: OpenCV was not found.")
    ENDIF(OpenCV_FIND_REQUIRED)
  ENDIF(NOT OpenCV_FIND_QUIETLY)
ENDIF(NOT OpenCV_FOUND)

# make FIND_PACKAGE case sensitive compatible
SET(OpenCV_FOUND       ${OPENCV_FOUND})
SET(OpenCV_LIBRARIES   ${OPENCV_LIBRARIES})
SET(OpenCV_INCLUDE_DIR ${OPENCV_INCLUDE_DIR})

## - Try to find OpenCV library installation
## See http://sourceforge.net/projects/opencvlibrary/
##
## The follwoing variables are optionally searched for defaults
##  OpenCV_ROOT_DIR:            Base directory of OpenCv tree to use.
##  OpenCV_FIND_REQUIRED_COMPONENTS : FIND_PACKAGE(OpenCV COMPONENTS ..) 
##    compatible interface. typically  CV CXCORE CVAUX HIGHGUI CVCAM .. etc.
##
## The following are set after configuration is done: 
##  OpenCV_FOUND
##  OpenCV_INCLUDE_DIR
##  OpenCV_LIBRARIES
##  OpenCV_LINK_DIRECTORIES
##
## deprecated:
##  OPENCV_* uppercase replaced by case sensitive OpenCV_*
##  OPENCV_EXE_LINKER_FLAGS
##  OPENCV_INCLUDE_DIR : replaced by plural *_DIRS
## 
## 2004/05 Jan Woetzel, Friso, Daniel Grest 
## 2006/01 complete rewrite by Jan Woetzel
## 1006/09 2nd rewrite introducing ROOT_DIR and PATH_SUFFIXES 
##   to handle multiple installed versions gracefully by Jan Woetzel
##
## tested with:
## -OpenCV 0.97 (beta5a):  MSVS 7.1, gcc 3.3, gcc 4.1
## -OpenCV 0.99 (1.0rc1):  MSVS 7.1
##
## www.mip.informatik.uni-kiel.de/~jw
## --------------------------------
#
#
#MACRO(DBG_MSG _MSG)
#  #  MESSAGE(STATUS "${CMAKE_CURRENT_LIST_FILE}(${CMAKE_CURRENT_LIST_LINE}):\n${_MSG}")
#ENDMACRO(DBG_MSG)
#
#
#
## required cv components with header and library if COMPONENTS unspecified
#IF    (NOT OpenCV_FIND_COMPONENTS)
#  # default
#  SET(OpenCV_FIND_REQUIRED_COMPONENTS   CV CXCORE CVAUX HIGHGUI )
##  IF   (WIN32)
##    LIST(APPEND OpenCV_FIND_REQUIRED_COMPONENTS  CVCAM ) # WIN32 only actually
##  ENDIF(WIN32)  
#ENDIF (NOT OpenCV_FIND_COMPONENTS)
#
#FILE(GLOB OPENCV2_PATHS "$ENV{ProgramFiles}/../OpenCV*.*")
#
## typical root dirs of installations, exactly one of them is used
#SET (OpenCV_POSSIBLE_ROOT_DIRS
#  "${OpenCV_ROOT_DIR}"
#  "$ENV{OpenCV_ROOT_DIR}"  
#  "$ENV{OPENCV_DIR}"  # only for backward compatibility deprecated by ROOT_DIR
#  "$ENV{OPENCV_HOME}" # only for backward compatibility
#  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]"
#  "$ENV{ProgramFiles}/OpenCV"
#  "${OPENCV2_PATHS}"
#  /usr/local
#  /usr
#  )
#
#
## MIP Uni Kiel /opt/net network installation 
## get correct prefix for current gcc compiler version for gcc 3.x  4.x
#IF    (${CMAKE_COMPILER_IS_GNUCXX})
#  IF    (NOT OpenCV_FIND_QUIETLY)
#    MESSAGE(STATUS "Checking GNUCXX version 3/4 to determine  OpenCV /opt/net/ path")
#  ENDIF (NOT OpenCV_FIND_QUIETLY)
#  EXEC_PROGRAM(${CMAKE_CXX_COMPILER} ARGS --version OUTPUT_VARIABLE CXX_COMPILER_VERSION)  
#  IF   (CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")
#    SET(IS_GNUCXX3 TRUE)
#    LIST(APPEND OpenCV_POSSIBLE_ROOT_DIRS /opt/net/gcc33/OpenCV )
#  ENDIF(CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")  
#  IF   (CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")
#    SET(IS_GNUCXX4 TRUE)
#    LIST(APPEND OpenCV_POSSIBLE_ROOT_DIRS /opt/net/gcc41/OpenCV )
#  ENDIF(CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")
#ENDIF (${CMAKE_COMPILER_IS_GNUCXX})
#
##DBG_MSG("DBG (OpenCV_POSSIBLE_ROOT_DIRS=${OpenCV_POSSIBLE_ROOT_DIRS}")
#
##
## select exactly ONE OpenCV base directory/tree 
## to avoid mixing different version headers and libs
##
#FIND_PATH(OpenCV_ROOT_DIR 
#  NAMES 
#  cv/include/cv.h     # windows
#  include/opencv/cv.h # linux /opt/net
#  include/cv/cv.h 
#  include/cv.h 
#  PATHS ${OpenCV_POSSIBLE_ROOT_DIRS})
#DBG_MSG("OpenCV_ROOT_DIR=${OpenCV_ROOT_DIR}")
#
#
## header include dir suffixes appended to OpenCV_ROOT_DIR
#SET(OpenCV_INCDIR_SUFFIXES
#  include
#  include/cv
#  include/opencv
#  cv/include
#  cxcore/include
#  cvaux/include
#  otherlibs/cvcam/include
#  otherlibs/highgui
#  otherlibs/highgui/include
#  otherlibs/_graphics/include
#  )
#
## library linkdir suffixes appended to OpenCV_ROOT_DIR 
#SET(OpenCV_LIBDIR_SUFFIXES
#  lib
#  OpenCV/lib
#  otherlibs/_graphics/lib
#  )
##DBG_MSG("OpenCV_LIBDIR_SUFFIXES=${OpenCV_LIBDIR_SUFFIXES}")
#
#
##
## find incdir for each lib
##
#FIND_PATH(OpenCV_CV_INCLUDE_DIR
#  NAMES cv.h      
#  PATHS ${OpenCV_ROOT_DIR} 
#  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )
#FIND_PATH(OpenCV_CXCORE_INCLUDE_DIR   
#  NAMES cxcore.h
#  PATHS ${OpenCV_ROOT_DIR} 
#  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )
#FIND_PATH(OpenCV_CVAUX_INCLUDE_DIR    
#  NAMES cvaux.h
#  PATHS ${OpenCV_ROOT_DIR} 
#  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )
#FIND_PATH(OpenCV_HIGHGUI_INCLUDE_DIR  
#  NAMES highgui.h 
#  PATHS ${OpenCV_ROOT_DIR} 
#  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )
##FIND_PATH(OpenCV_CVCAM_INCLUDE_DIR    
##  NAMES cvcam.h 
##  PATHS ${OpenCV_ROOT_DIR} 
##  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} )
#
##
## find sbsolute path to all libraries 
## some are optionally, some may not exist on Linux
##
#  
#FIND_LIBRARY(OpenCV_CV_LIBRARY   
#  NAMES cv opencv cv200 cv210
#  PATHS ${OpenCV_ROOT_DIR}  
#  PATH_SUFFIXES  ${OpenCV_LIBDIR_SUFFIXES} )
#FIND_LIBRARY(OpenCV_CVAUX_LIBRARY
#  NAMES cvaux cvaux200 cvaux210
#  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
##FIND_LIBRARY(OpenCV_CVCAM_LIBRARY   
##  NAMES cvcam
##  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} ) 
##FIND_LIBRARY(OpenCV_CVHAARTRAINING_LIBRARY
##  NAMES cvhaartraining
##  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} ) 
#FIND_LIBRARY(OpenCV_CXCORE_LIBRARY  
#  NAMES cxcore cxcore200 cxcore210
#  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
##FIND_LIBRARY(OpenCV_CXTS_LIBRARY   
##  NAMES cxts
##  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
#FIND_LIBRARY(OpenCV_HIGHGUI_LIBRARY  
#  NAMES highgui highgui200 highgui200d
#  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
##FIND_LIBRARY(OpenCV_ML_LIBRARY  
##  NAMES ml
##  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
##FIND_LIBRARY(OpenCV_TRS_LIBRARY  
##  NAMES trs
##  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
#
#
##
## Logic selecting required libs and headers
##
#SET(OpenCV_FOUND ON)
#DBG_MSG("OpenCV_FIND_REQUIRED_COMPONENTS=${OpenCV_FIND_REQUIRED_COMPONENTS}")
#FOREACH(NAME ${OpenCV_FIND_REQUIRED_COMPONENTS} )
#
#  # only good if header and library both found   
#  IF    (OpenCV_${NAME}_INCLUDE_DIR AND OpenCV_${NAME}_LIBRARY)
#    LIST(APPEND OpenCV_INCLUDE_DIRS ${OpenCV_${NAME}_INCLUDE_DIR} )
#    LIST(APPEND OpenCV_LIBRARIES    ${OpenCV_${NAME}_LIBRARY} )
#    #DBG_MSG("appending for NAME=${NAME} ${OpenCV_${NAME}_INCLUDE_DIR} and ${OpenCV_${NAME}_LIBRARY}" )
#  ELSE  (OpenCV_${NAME}_INCLUDE_DIR AND OpenCV_${NAME}_LIBRARY)
#    DBG_MSG("OpenCV component NAME=${NAME} not found! "
#      "\nOpenCV_${NAME}_INCLUDE_DIR=${OpenCV_${NAME}_INCLUDE_DIR} "
#      "\nOpenCV_${NAME}_LIBRARY=${OpenCV_${NAME}_LIBRARY} ")
#    SET(OpenCV_FOUND OFF)
#  ENDIF (OpenCV_${NAME}_INCLUDE_DIR AND OpenCV_${NAME}_LIBRARY)
#  
#ENDFOREACH(NAME)
#
#DBG_MSG("OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
#DBG_MSG("OpenCV_LIBRARIES=${OpenCV_LIBRARIES}")
#
## get the link directory for rpath to be used with LINK_DIRECTORIES: 
#IF    (OpenCV_CV_LIBRARY)
#  GET_FILENAME_COMPONENT(OpenCV_LINK_DIRECTORIES ${OpenCV_CV_LIBRARY} PATH)
#ENDIF (OpenCV_CV_LIBRARY)
#
#MARK_AS_ADVANCED(
#  OpenCV_ROOT_DIR
#  OpenCV_INCLUDE_DIRS
#  OpenCV_CV_INCLUDE_DIR
#  OpenCV_CXCORE_INCLUDE_DIR
#  OpenCV_CVAUX_INCLUDE_DIR
#  OpenCV_CVCAM_INCLUDE_DIR
#  OpenCV_HIGHGUI_INCLUDE_DIR
#  OpenCV_LIBRARIES
#  OpenCV_CV_LIBRARY
#  OpenCV_CXCORE_LIBRARY
#  OpenCV_CVAUX_LIBRARY
#  OpenCV_CVCAM_LIBRARY
#  OpenCV_CVHAARTRAINING_LIBRARY
#  OpenCV_CXTS_LIBRARY
#  OpenCV_HIGHGUI_LIBRARY
#  OpenCV_ML_LIBRARY
#  OpenCV_TRS_LIBRARY
#  )
#
#
## be backward compatible:
#SET(OPENCV_LIBRARIES   ${OpenCV_LIBRARIES} )
#SET(OPENCV_INCLUDE_DIR ${OpenCV_INCLUDE_DIRS} )
#SET(OPENCV_FOUND       ${OpenCV_FOUND})
#SET(OpenCV_LIBS        ${OpenCV_LIBRARIES} )
##SET(OpenCV_INCLUDE_DIRS )
##SET(OpenCV_LIBRARY_DIRS )
#
#
#
## display help message
#IF(NOT OpenCV_FOUND)
#  # make FIND_PACKAGE friendly
#  IF(NOT OpenCV_FIND_QUIETLY)
#    IF(OpenCV_FIND_REQUIRED)
#      MESSAGE(FATAL_ERROR
#        "OpenCV required but some headers or libs not found. Please specify it's location with OpenCV_ROOT_DIR env. variable.")
#    ELSE(OpenCV_FIND_REQUIRED)
#      MESSAGE(STATUS 
#        "ERROR: OpenCV was not found.")
#    ENDIF(OpenCV_FIND_REQUIRED)
#  ENDIF(NOT OpenCV_FIND_QUIETLY)
#ENDIF(NOT OpenCV_FOUND)
