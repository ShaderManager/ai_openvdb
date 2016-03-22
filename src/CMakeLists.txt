set(sources plugin.cpp
			volume.cpp volume.hpp
			utils.cpp utils.hpp)

add_library(ai_openvdb SHARED ${sources})

target_compile_features(ai_openvdb PRIVATE cxx_auto_type cxx_decltype cxx_trailing_return_types cxx_uniform_initialization)

if (WIN32)
	target_compile_definitions(ai_openvdb PRIVATE -D__TBB_NO_IMPLICIT_LINKAGE)
endif ()

target_include_directories(ai_openvdb PRIVATE ${ARNOLD_INCLUDE_DIRS} ${OPENVDB_INCLUDE_DIRS})
target_include_directories(ai_openvdb PRIVATE ${Boost_INCLUDE_DIRS} ${ILMBASE_INCLUDE_DIR} ${TBB_INCLUDE_DIR})

target_link_libraries(ai_openvdb ${ARNOLD_LIBRARIES} ${OPENVDB_LIBRARIES} ${TBB_LIBRARIES} ${ILMBASE_HALF_LIBRARY})