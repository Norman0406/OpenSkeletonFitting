# Replace INTDIR with the value of $ENV{CMAKE_CONFIG_TYPE} that is set
# by ctest when -C Debug|Releaes|etc is given, and INDIR is passed
# in from the main cmake run and is the variable that is used
# by the build system to specify the build directory
if(NOT "${INTDIR}" STREQUAL ".")
  set(TEST_ORIG "${TEST}")
  string(REPLACE "${INTDIR}" "$ENV{CMAKE_CONFIG_TYPE}" TEST "${TEST}")
  if("$ENV{CMAKE_CONFIG_TYPE}" STREQUAL "")
    if(NOT EXISTS "${TEST}")
      message("Warning: CMAKE_CONFIG_TYPE not defined did you forget the -C option for ctest?")
      message(FATAL_ERROR "Could not find test executable: ${TEST_ORIG}")
    endif()
  endif()
endif()
set(ARGS )
set(ARGS OUTPUT_FILE "${OUTPUT}"  ERROR_FILE "${OUTPUT}.err")
message("Running: ${TEST}")
message("ARGS= ${ARGS}")
execute_process(COMMAND "${TEST}" 
  ${ARGS}
  RESULT_VARIABLE RET)
# if the test does not return 0, then fail it
if(NOT ${RET} EQUAL 0)
  message(FATAL_ERROR "Test ${TEST} returned ${RET}")
endif()
# result with reference
execute_process(COMMAND "${CMAKE_COMMAND}"
  -E compare_files  "${OUTPUT}" "${REFERENCE}"
  RESULT_VARIABLE RET)
# if the test does not return 0, then fail it
if(NOT ${RET} EQUAL 0)
  message(FATAL_ERROR "Test ${TEST} produced a result which is different from the reference}")
endif()
message( "Test ${TEST} returned ${RET}")
