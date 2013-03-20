include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/Externals/include)

set(SOURCEFILES
	System.cpp
)

set(HEADERFILES
	System.h
)

add_library(OpenSFLib ${SOURCEFILES} ${HEADERFILES})