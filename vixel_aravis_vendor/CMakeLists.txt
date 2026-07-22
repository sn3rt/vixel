cmake_minimum_required(VERSION 3.15)
project(vixel_aravis_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ARAVIS QUIET aravis-0.8>=0.8.34)

add_library(vixel_aravis_vendor INTERFACE)
if(ARAVIS_FOUND)
  message(STATUS "Using system Aravis ${ARAVIS_VERSION}")
  target_include_directories(vixel_aravis_vendor INTERFACE ${ARAVIS_INCLUDE_DIRS})
  target_link_directories(vixel_aravis_vendor INTERFACE ${ARAVIS_LIBRARY_DIRS})
  target_link_libraries(vixel_aravis_vendor INTERFACE ${ARAVIS_LINK_LIBRARIES})
  target_compile_options(vixel_aravis_vendor INTERFACE ${ARAVIS_CFLAGS_OTHER})
else()
  find_program(MESON_EXECUTABLE meson REQUIRED)
  pkg_check_modules(ARAVIS_BASE REQUIRED glib-2.0 gio-2.0 gobject-2.0 libxml-2.0)
  set(_aravis_prefix "${CMAKE_INSTALL_PREFIX}")
  set(_aravis_library "${_aravis_prefix}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}aravis-0.8${CMAKE_SHARED_LIBRARY_SUFFIX}")
  file(MAKE_DIRECTORY "${_aravis_prefix}/include/aravis-0.8")
  ExternalProject_Add(aravis_external
    GIT_REPOSITORY https://github.com/AravisProject/aravis.git
    GIT_TAG 0.8.36
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
    CONFIGURE_COMMAND
      ${MESON_EXECUTABLE} setup <BINARY_DIR> <SOURCE_DIR>
        --prefix=${_aravis_prefix} --libdir=lib --buildtype=release
        -Dviewer=disabled -Dgst-plugin=disabled -Dintrospection=disabled
        -Ddocumentation=disabled
    BUILD_COMMAND ${MESON_EXECUTABLE} compile -C <BINARY_DIR>
    INSTALL_COMMAND ${MESON_EXECUTABLE} install -C <BINARY_DIR>
    BUILD_BYPRODUCTS "${_aravis_library}")
  target_include_directories(vixel_aravis_vendor INTERFACE
    "${_aravis_prefix}/include/aravis-0.8" ${ARAVIS_BASE_INCLUDE_DIRS})
  target_link_libraries(vixel_aravis_vendor INTERFACE
    "${_aravis_library}" ${ARAVIS_BASE_LINK_LIBRARIES})
  add_dependencies(vixel_aravis_vendor aravis_external)
  message(STATUS "Compatible system Aravis not found; pinned Aravis 0.8.36 will be built locally")
endif()

install(TARGETS vixel_aravis_vendor EXPORT vixel_aravis_vendorTargets)
ament_export_targets(vixel_aravis_vendorTargets HAS_LIBRARY_TARGET)
ament_package()
