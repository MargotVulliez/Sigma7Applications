# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SIGMA7_APPLICATIONS_BINARY_DIR}/01-WorskpaceExtensionSimu)
ADD_EXECUTABLE (controller01 controller.cpp ${SIGMA7_APPLICATIONS_COMMON_SOURCE})
ADD_EXECUTABLE (simviz01 simviz.cpp ${SIGMA7_APPLICATIONS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller01 ${SIGMA7_APPLICATIONS_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz01 ${SIGMA7_APPLICATIONS_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${SIGMA7_APPLICATIONS_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/01-WorskpaceExtensionSimu)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY kuka_iiwa.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY proxy.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY proxy6d.urdf DESTINATION ${APP_RESOURCE_DIR})
# FILE(COPY kuka_iiwa_dremel.urdf DESTINATION ${APP_RESOURCE_DIR})
