# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

#change this to your own rainmaker path
set(RMAKER_PATH C:/Users/hello/esp-rainmaker)

# Add RainMaker components and other common application components
set(EXTRA_COMPONENT_DIRS 
${RMAKER_PATH}/components/esp-insights/components ${RMAKER_PATH}/components ${RMAKER_PATH}/examples/common ${CMAKE_SOURCE_DIR}/components/esp-idf-lib/components 
)

set(PROJECT_VER "1.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(temperature_sensor)
