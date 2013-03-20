include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/Externals/include)

set(SOURCEFILES
	Exception.cpp
	Logging.cpp
	Module.cpp
	ModuleInput.cpp
	ModuleOutput.cpp
	Timer.cpp
	Utils.cpp
)

set(HEADERFILES
	Exception.h
	Factory.h
	Logging.h
	Module.h
	ModuleInput.h
	ModuleOutput.h
	Timer.h
	Utils.h
)

add_library(OpenSF ${SOURCEFILES} ${HEADERFILES})