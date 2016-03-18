include_directories(${ARNOLD_INCLUDE_DIRS} ${OPENVDB_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS} ${ILMBASE_INCLUDE_DIR} ${TBB_INCLUDE_DIR})

if (WIN32)
	add_definitions(-D__TBB_NO_IMPLICIT_LINKAGE)
endif ()

set(sources plugin.cpp)

add_library(ai_openvdb SHARED ${sources})

target_link_libraries(ai_openvdb ${ARNOLD_LIBRARIES} ${OPENVDB_LIBRARIES} ${TBB_LIBRARIES} ${ILMBASE_HALF_LIBRARY})