cmake_minimum_required ( VERSION 2.6 )


set( CMAKE_VERBOSE OFF CACHE BOOL "Verbose mode" )

if ( CMAKE_VERBOSE )
        set ( CMAKE_VERBOSE_MAKEFILE 1 )
endif( )

if( CMAKE_CACHE_MAJOR_VERSION>=2 AND CMAKE_CACHE_MINOR_VERSION>6 )
	cmake_policy( SET CMP0015 OLD )
endif( )

project ( opencvSfM )
add_definitions( -DSFM_API_EXPORTS )

if( MSVC )
    set( CMAKE_USE_RELATIVE_PATHS ON CACHE INTERNAL "" FORCE )
endif( )
# --------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
#  and absolute paths in linker lib lists.
# Run "cmake --help-policy CMP0003" for more information.
# --------------------------------------------------------------
if( COMMAND cmake_policy )
    cmake_policy( SET CMP0003 NEW )
endif( )


# ----------------------------------------------------------------------------
#  Current version number:
# ----------------------------------------------------------------------------
set( SfM_lib_VERSION "0.1.0" )

string( REGEX MATCHALL "[ 0-9 ]" SfM_lib_VERSION_PARTS "${SfM_lib_VERSION}" )

list( GET SfM_lib_VERSION_PARTS 0 SfM_lib_VERSION_MAJOR )
list( GET SfM_lib_VERSION_PARTS 1 SfM_lib_VERSION_MINOR )
list( GET SfM_lib_VERSION_PARTS 2 SfM_lib_VERSION_PATCH )
set( SfM_lib_SOVERSION "${SfM_lib_VERSION_MAJOR}.${SfM_lib_VERSION_MINOR}" )

if( WIN32 )
    # Postfix of DLLs:
    set( SfM_lib_DLLVERSION "${SfM_lib_VERSION_MAJOR}${SfM_lib_VERSION_MINOR}${SfM_lib_VERSION_PATCH}" )
    set( SfM_lib_DEBUG_POSTFIX d )
else( )
    # Postfix of so's:
    #set( SfM_lib_DLLVERSION "${SfM_lib_VERSION_MAJOR}${SfM_lib_VERSION_MINOR}${SfM_lib_VERSION_PATCH}" )
    set( SfM_lib_DLLVERSION "" )
    set( SfM_lib_DEBUG_POSTFIX )
endif( )

include( LibraryAdd.cmake REQUIRED )
include( ProjectAdd.cmake REQUIRED )

FIND_PACKAGE( OpenCV REQUIRED )

# Unfortunately boost puts the version number into the
# actual filename for the libraries, so this variable will certainly be needed
# in the future when new Boost versions are released.
SET( Boost_ADDITIONAL_VERSIONS "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46.1" "1.46.0" "1.46" "1.47" )

find_package( Boost COMPONENTS filesystem system )
if( Boost_FOUND )
	add_definitions( -DHAVE_BOOST )
	LINK_DIRECTORIES( ${Boost_LIBRARY_DIRS} )
	include_directories( ${Boost_INCLUDE_DIR} )
else( )
	find_path( BOOST_ROOT "boost"
	DOC "The path to Boost root directory ( before boost_1_XX_X )" )
	message( STATUS " ** Warning: Boost library Could NOT be found, you will not be able to process automatically a folder full of images /!\\" )
endif( )

find_path( EIGEN_DIR "Eigen/Core"
	PATHS "/usr/local/include/" "/opt/include/" "/usr/include/"
	PATH_SUFFIXES "eigen2" "eigen3" "eigen"
	DOC "The path to Eigen2/Eigen3 headers" )
#Generate the source list:
#include_directories( "${PROJECT_SOURCE_DIR}/src" )
#include_directories( "${PROJECT_SOURCE_DIR}/tutorials" )
include( CheckIncludeFiles )
check_include_files( "pthread.h" HAVE_PTHREAD_H )

set( SFM_LINKER_LIBS ${OpenCV_LIBS} ${Boost_LIBRARIES} )

if( ${HAVE_PTHREAD_H} )
	find_package( Threads )
	set( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${CMAKE_THREAD_LIBS_INIT} )
endif( )

include_directories( ${EIGEN_DIR} )

find_package( PCL 1.1 )

set( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${PCL_LIBRARIES} )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

#find_package( libmv ) not possible now :(
find_path( LIBMV_HEADER_DIR "libmv/multiview"
	PATH_SUFFIXES "src"
	DOC "The path to LIBMV headers" )
find_library( LIBMV_LIB_DIR "multiview"
	PATHS ${LIBMV_HEADER_DIR} "../${LIBMV_HEADER_DIR}"
	PATH_SUFFIXES "lib"
	DOC "The path to LIBMV lib" )
if( WIN32 )
SET ( LIBMV_LIB debug "multiview_d" "numeric_d" "V3D" "ldl" "colamd" optimized "multiview" "numeric" "V3D" "ldl" "colamd" )
else( )
SET ( LIBMV_LIB "libmultiview.a" "libnumeric.a" "libtools.a" "libV3D.a" "libldl.a" "libcolamd.a" "libglog.a" "libgflags.a" "libgtest.a" )
endif( )
include_directories( ${LIBMV_HEADER_DIR} )
link_directories( ${LIBMV_LIB_DIR} )
SET( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${LIBMV_LIB} )

# Allow the developer to select if Dynamic or Static libraries are built
OPTION ( BUILD_STATIC_LIBS "Build Static Libraries" OFF )
# Set the LIB_TYPE variable to SHARED
SET ( LIB_TYPE SHARED )
IF ( BUILD_STATIC_LIBS )
  # User wants to build Static Libraries, so change the LIB_TYPE variable to CMake keyword 'STATIC'
  SET ( LIB_TYPE STATIC )
ENDIF ( BUILD_STATIC_LIBS )

configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_BINARY_DIR}/config.h IMMEDIATE )
add_subdirectory( src )
add_subdirectory( tutorials )
