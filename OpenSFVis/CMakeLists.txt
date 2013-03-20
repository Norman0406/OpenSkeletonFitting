include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/Externals/include)
include_directories(${PROJECT_SOURCE_DIR}/Externals/include/openni)
link_directories(${PROJECT_SOURCE_DIR}/Externals/lib)

IF(APPLE)
	link_directories(/opt/local/lib)
	find_library(OpenGL_LIBRARY OpenGL)
	include_directories(${OPENGL_INCLUDE_DIR})
ENDIF(APPLE)

set(SOURCEFILES
	main.cpp
	Visualization2d.cpp
	Visualization3d.cpp
)

set(HEADERFILES
	Visualization2d.h
	Visualization3d.h
)

add_executable(OpenSFVis ${SOURCEFILES} ${HEADERFILES})
target_link_libraries(OpenSFVis ${PROJECTLIBS})

#add external libraries
target_link_libraries(OpenSFVis flann GLFW openNI)

IF(APPLE)
	target_link_libraries(OpenSFVis ${OpenGL_LIBRARY})
ENDIF(APPLE)

target_link_libraries(OpenSFVis
	opencv_calib3d231
	opencv_core231
	opencv_highgui231
	opencv_imgproc231
	opencv_video231
	boost_thread-mt)