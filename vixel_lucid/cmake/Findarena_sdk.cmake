set(ARENA_SDK_ROOT "" CACHE PATH "Root of a LUCID Arena SDK installation")

if(NOT ARENA_SDK_ROOT AND DEFINED ENV{ARENA_SDK_ROOT})
  set(ARENA_SDK_ROOT "$ENV{ARENA_SDK_ROOT}")
endif()

if(NOT ARENA_SDK_ROOT)
  set(_arena_sdk_conf "/etc/ld.so.conf.d/Arena_SDK.conf")
  if(EXISTS "${_arena_sdk_conf}")
    file(STRINGS "${_arena_sdk_conf}" _arena_library_paths LIMIT_COUNT 1)
    list(GET _arena_library_paths 0 _arena_library_path)
    get_filename_component(ARENA_SDK_ROOT "${_arena_library_path}" DIRECTORY)
  endif()
endif()

set(_arena_library_dirs "${ARENA_SDK_ROOT}/lib64" "${ARENA_SDK_ROOT}/lib")
set(_genicam_library_dirs
  "${ARENA_SDK_ROOT}/GenICam/library/lib/Linux64_x64"
  "${ARENA_SDK_ROOT}/GenICam/library/lib/Linux64_ARM"
)
find_path(arena_sdk_ARENA_INCLUDE_DIR NAMES ArenaApi.h
  PATHS "${ARENA_SDK_ROOT}/include/Arena" NO_DEFAULT_PATH)
find_path(arena_sdk_GENICAM_INCLUDE_DIR NAMES GenICam.h
  PATHS "${ARENA_SDK_ROOT}/GenICam/library/CPP/include" NO_DEFAULT_PATH)
find_library(arena_sdk_ARENA_LIBRARY NAMES arena PATHS ${_arena_library_dirs} NO_DEFAULT_PATH)
find_library(arena_sdk_SAVE_LIBRARY NAMES save PATHS ${_arena_library_dirs} NO_DEFAULT_PATH)
find_library(arena_sdk_GENTL_LIBRARY NAMES gentl PATHS ${_arena_library_dirs} NO_DEFAULT_PATH)
find_library(arena_sdk_GCBASE_LIBRARY NAMES GCBase_gcc54_v3_3_LUCID GCBase_gcc421_v3_0
  PATHS ${_genicam_library_dirs} NO_DEFAULT_PATH)
find_library(arena_sdk_GENAPI_LIBRARY NAMES GenApi_gcc54_v3_3_LUCID GenApi_gcc421_v3_0
  PATHS ${_genicam_library_dirs} NO_DEFAULT_PATH)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(arena_sdk REQUIRED_VARS
  ARENA_SDK_ROOT arena_sdk_ARENA_INCLUDE_DIR arena_sdk_GENICAM_INCLUDE_DIR
  arena_sdk_ARENA_LIBRARY arena_sdk_SAVE_LIBRARY arena_sdk_GENTL_LIBRARY
  arena_sdk_GCBASE_LIBRARY arena_sdk_GENAPI_LIBRARY)
if(arena_sdk_FOUND)
  set(arena_sdk_INCLUDE_DIRS "${arena_sdk_ARENA_INCLUDE_DIR}" "${arena_sdk_GENICAM_INCLUDE_DIR}")
  set(arena_sdk_LIBRARIES "${arena_sdk_ARENA_LIBRARY}" "${arena_sdk_SAVE_LIBRARY}"
    "${arena_sdk_GENTL_LIBRARY}" "${arena_sdk_GCBASE_LIBRARY}" "${arena_sdk_GENAPI_LIBRARY}")
  message(STATUS "Arena SDK root: ${ARENA_SDK_ROOT}")
endif()
