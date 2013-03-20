include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/Externals/include)
include_directories(${PROJECT_SOURCE_DIR}/Externals/include/openni)
link_directories(${PROJECT_SOURCE_DIR}/Externals/lib)

#only on mac
link_directories(/opt/local/lib)

set(SOURCEFILES
	main.cpp
)

set(HEADERFILES
)

add_executable(OpenSFApp ${SOURCEFILES} ${HEADERFILES})
target_link_libraries(OpenSFApp ${PROJECTLIBS})

#add external libraries
target_link_libraries(OpenSFApp flann openNI)

target_link_libraries(OpenSFApp
	opencv_calib3d231
	opencv_core231
	opencv_highgui231
	opencv_imgproc231
	opencv_video231
	boost_thread-mt)