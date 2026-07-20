set(PROJECT_NAME CSC8503CoreClasses)

################################################################################
# Source groups
################################################################################

file(GLOB_RECURSE Header_Files CONFIGURE_DEPENDS *.h)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${Header_Files})

file(GLOB_RECURSE Source_Files CONFIGURE_DEPENDS *.cpp *.c)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${Source_Files})

#file(GLOB_RECURSE Source_Files CONFIGURE_DEPENDS *.c)
#source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${Source_Files})

set(ALL_FILES
    ${Header_Files}
    ${Source_Files}
)

set_source_files_properties(${ALL_FILES} PROPERTIES LANGUAGE CXX)

################################################################################
# Target
################################################################################
add_library(${PROJECT_NAME} STATIC ${ALL_FILES})
set(ROOT_NAMESPACE CSC8503CoreClasses)

target_precompile_headers(${PROJECT_NAME} PRIVATE    
	<fstream>
    <sstream>
	<iostream>
	<iosfwd>
	
    <memory>
	<string>
	<thread>
    <filesystem>
    <functional>
	<assert.h>  
	<chrono>	
	<functional>

	<random>
  
    <unordered_set>
	<set>
    <vector>
	<unordered_map>
    <map> 
	<stack> 

	
	"../NCLCoreClasses/Vector.h"
    "../NCLCoreClasses/Quaternion.h"
    "../NCLCoreClasses/Plane.h"
    "../NCLCoreClasses/Matrix.h"

	"../NCLCoreClasses/Window.h"
)

include_directories("../NCLCoreClasses/")
include_directories("./")

if(MSVC)
	target_link_libraries(${PROJECT_NAME} PRIVATE "winmm.lib")
    target_link_libraries(${PROJECT_NAME} PRIVATE "ws2_32.lib")
endif()