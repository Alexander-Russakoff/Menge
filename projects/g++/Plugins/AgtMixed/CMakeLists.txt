cmake_minimum_required(VERSION 2.8)

project(MixedModel)

add_definitions( -DMIXED_EXPORT )

file(
	GLOB_RECURSE
	source_files
	${PLUGIN_SOURCE_DIR}/AgtMixed/*.cpp
	${PLUGIN_SOURCE_DIR}/AgtMixed/*.h
)

add_library(
	mixedPed
	SHARED
	${source_files}
)

target_link_libraries (mixedPed mengeCore )
