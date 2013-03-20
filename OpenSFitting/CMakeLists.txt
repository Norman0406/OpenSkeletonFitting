include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/Externals/include)

set(SOURCEFILES
	Constraint.cpp
	Fitting.cpp
	Joint.cpp
	Skeleton.cpp
	SkeletonCommon.cpp
	SkeletonFitting.cpp
	SkeletonHuman.cpp
	SkeletonManipulator.cpp
)

set(HEADERFILES
	Constraint.h
	Fitting.h
	FittingFactory.h
	Joint.h
	Skeleton.h
	SkeletonCommon.h
	SkeletonFitting.h
	SkeletonHuman.h
	SkeletonManipulator.h
)

add_library(OpenSFitting ${SOURCEFILES} ${HEADERFILES})