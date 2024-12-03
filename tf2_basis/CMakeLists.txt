cmake_minimum_required(VERSION 3.5)
project(tf2_basis LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()


if(PROJECT_IS_TOP_LEVEL)
include(FetchContent)
FetchContent_Declare(
    basis
    GIT_REPOSITORY https://github.com/basis-robotics/basis.git
    GIT_TAG main
)
FetchContent_MakeAvailable(basis)
endif()

if(NOT BASIS_SOURCE_ROOT)
  message( FATAL_ERROR "Basis is missing!" )
endif()

set(TF2_DIR $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../tf2>)

#CPP Libraries
add_library(tf2 ${TF2_DIR}/src/cache.cpp ${TF2_DIR}/src/buffer_core.cpp ${TF2_DIR}/src/static_cache.cpp ${TF2_DIR}/src/time.cpp)
target_include_directories(tf2 PUBLIC include_ros "${TF2_DIR}/include")

add_library(tf2_basis src/tf2_basis.cpp)

target_link_libraries(tf2_basis tf2 basis::core::time foxglove_schemas_protobuf)
target_include_directories(tf2_basis PUBLIC include)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(tf2 PRIVATE "TF2_BUILDING_DLL")

if(false)
install(TARGETS tf2 EXPORT export_tf2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
endif()