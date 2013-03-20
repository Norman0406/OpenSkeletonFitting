include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/Externals/include)

set(SOURCEFILES
	Background.cpp
	BackgroundFirst.cpp
	Segmentation.cpp
	SegmentationBackground.cpp
)

set(HEADERFILES
	Background.h
	BackgroundFactory.h
	BackgroundFirst.h
	Segmentation.h
	SegmentationBackground.h
	SegmentationFactory.h
)

add_library(OpenSFSegmentation ${SOURCEFILES} ${HEADERFILES})