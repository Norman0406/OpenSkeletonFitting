cmake_minimum_required(VERSION 2.6)
project(OpenSkeletonFitting)
include_directories("${PROJECT_BINARY_DIR}")

IF(APPLE)
	include_directories("/opt/local/include/")
ENDIF(APPLE)

# NOTE: what about precompiled headers?

# add CMinPack library
include_directories("${PROJECT_SOURCE_DIR}/CMinPack")
add_subdirectory(CMinPack)
set(PROJECTLIBS ${PROJECTLIBS} cminpack)

# add OpenSF library
include_directories("${PROJECT_SOURCE_DIR}/OpenSF")
add_subdirectory(OpenSF)
set(PROJECTLIBS ${PROJECTLIBS} OpenSF)

# add OpenSFFeatures library
include_directories("${PROJECT_SOURCE_DIR}/OpenSFFeatures")
add_subdirectory(OpenSFFeatures)
set(PROJECTLIBS ${PROJECTLIBS} OpenSFFeatures)

# add OpenSFInput library
include_directories("${PROJECT_SOURCE_DIR}/OpenSFInput")
add_subdirectory(OpenSFInput)
set(PROJECTLIBS ${PROJECTLIBS} OpenSFInput)

# add OpenSFitting library
include_directories("${PROJECT_SOURCE_DIR}/OpenSFitting")
add_subdirectory(OpenSFitting)
set(PROJECTLIBS ${PROJECTLIBS} OpenSFitting)

# add OpenSFLib library
include_directories("${PROJECT_SOURCE_DIR}/OpenSFLib")
add_subdirectory(OpenSFLib)
set(PROJECTLIBS ${PROJECTLIBS} OpenSFLib)

# add OpenSFSegmentation library
include_directories("${PROJECT_SOURCE_DIR}/OpenSFSegmentation")
add_subdirectory(OpenSFSegmentation)
set(PROJECTLIBS ${PROJECTLIBS} OpenSFSegmentation)

# add OpenSFApp executable
include_directories("${PROJECT_SOURCE_DIR}/OpenSFApp")
add_subdirectory(OpenSFApp)

# add OpenSFVis executable
include_directories("${PROJECT_SOURCE_DIR}/OpenSFVis")
add_subdirectory(OpenSFVis)