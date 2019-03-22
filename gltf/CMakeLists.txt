# bump version here
set(gltf_VERSION 1.0)

define_module(LIBRARY gltf=${gltf_VERSION}
  DEPENDS math>=1.4 utility>=1.26 dbglog>=1.4 imgproc>=1.28
  Boost_FILESYSTEM Boost_IOSTREAMS
  )

set(gltf_SOURCES
  gltf.hpp gltf.cpp
  detail.hpp
  glb.cpp
  meshloader.hpp meshloader.cpp
)

add_library(gltf STATIC ${gltf_SOURCES})
buildsys_library(gltf)

target_link_libraries(gltf ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(gltf ${MODULE_DEFINITIONS})
