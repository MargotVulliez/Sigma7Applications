# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SIGMA7_APPLICATIONS_BINARY_DIR}/14-ExperimentStiffnessVariation)
ADD_EXECUTABLE (app14 app.cpp  ${SIGMA7_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (app14 ${SIGMA7_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY sphere.urdf DESTINATION ${APP_RESOURCE_DIR})
