# bump version here
set(gltf_VERSION 1.1)

define_module(LIBRARY gltf=${gltf_VERSION}
  DEPENDS math>=1.5 utility>=1.41 dbglog>=1.4 imgproc>=1.28 jsoncpp>=2.3
  Boost_FILESYSTEM Boost_IOSTREAMS
  )

set(gltf_SOURCES
  gltf.hpp gltf.cpp
  detail.hpp
  detail/support.hpp detail/support.cpp
  glb.cpp
  validate.cpp
  meshloader.hpp meshloader.cpp
  v1/parse.hpp v1/parse.cpp
)

add_library(gltf STATIC ${gltf_SOURCES})
buildsys_library(gltf)

target_link_libraries(gltf ${MODULE_LIBRARIES})
target_compile_definitions(gltf PRIVATE ${MODULE_DEFINITIONS})
