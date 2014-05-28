###############################################
# Dynamics
PROJECT(utility)

# Search all header and source files
FILE(GLOB utility_srcs "*.cpp")
FILE(GLOB utility_hdrs "*.h")

# Library 
ADD_LIBRARY(utility ${utility_srcs} ${utility_hdrs})
SET_TARGET_PROPERTIES(utility PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
SET_TARGET_PROPERTIES(utility PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")

