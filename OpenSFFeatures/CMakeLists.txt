include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/Externals/include)

set(SOURCEFILES
	FeaturePoint.cpp
	Features.cpp
	FeatureTracking.cpp
	GeodesicDistMap.cpp
)

set(HEADERFILES
	FeaturePoint.h
	Features.h
	FeaturesFactory.h
	FeatureTracking.h
	GeodesicDistMap.h
)

add_library(OpenSFFeatures ${SOURCEFILES} ${HEADERFILES})