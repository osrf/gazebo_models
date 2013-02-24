macro(_list_to_string _string _list)
    set(${_string})
    foreach(_item ${_list})
        string(LENGTH "${${_string}}" _len)
        if(${_len} GREATER 0)
          set(${_string} "${${_string}} ${_item}")
        else(${_len} GREATER 0)
          set(${_string} "${_item}")
        endif(${_len} GREATER 0)
    endforeach(_item)
endmacro()


macro(invoke_rospack pkgname _prefix _varname)
  # Check that our cached location of rospack is valid.  It can be invalid
  # if rospack has moved since last time we ran, #1154.  If it's invalid,
  # search again.
  if(NOT EXISTS ${ROSPACK_EXE})
    message("Cached location of rospack is invalid; searching for rospack...")
    set(ROSPACK_EXE ROSPACK_EXE-NOTFOUND)
    # Only look in PATH for rospack, #3831
    find_program(ROSPACK_EXE NAMES rospack DOC "rospack executable" NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH)
    if (NOT ROSPACK_EXE)
      message(FATAL_ERROR "Couldn't find rospack. Please source the appropriate ROS configuration file (e.g., /opt/ros/fuerte/setup.sh)")
    endif(NOT ROSPACK_EXE)
  endif(NOT EXISTS ${ROSPACK_EXE})
  set(_rospack_invoke_result)
  execute_process(
    COMMAND ${ROSPACK_EXE} ${ARGN} ${pkgname}
    OUTPUT_VARIABLE _rospack_invoke_result
    ERROR_VARIABLE _rospack_err_ignore
    RESULT_VARIABLE _rospack_failed
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if (_rospack_failed)
    #set(_rospack_${_varname} "")
    #set(${_prefix}_${_varname} "" CACHE INTERNAL "")
    message("Failed to invoke ${ROSPACK_EXE} ${ARGN} ${pkgname}")
    message("${_rospack_err_ignore}")
    message("${_rospack_invoke_result}")
    message(FATAL_ERROR "\nFailed to invoke rospack to get compile flags for package '${pkgname}'.  Look above for errors from rospack itself.  Aborting.  Please fix the broken dependency!\n")
  else(_rospack_failed)
    separate_arguments(_rospack_invoke_result)
    set(_rospack_${_varname} ${_rospack_invoke_result})
    # We don't cache results that contain newlines, because
    # they make CMake's cache unhappy. This check should only affect calls
    # to `rospack plugins`, which we don't need to cache.
    if(_rospack_invoke_result MATCHES ".*\n.*")
      set(${_prefix}_${_varname} "${_rospack_invoke_result}")
    else(_rospack_invoke_result MATCHES ".*\n.*")
      set(${_prefix}_${_varname} "${_rospack_invoke_result}" CACHE INTERNAL "")
    endif(_rospack_invoke_result MATCHES ".*\n.*")
  endif(_rospack_failed)
endmacro()

macro(get_rospack_flags pkgname)
  # Get the include dirs
  set(_prefix ${pkgname})
  invoke_rospack(${pkgname} ${_prefix} "INCLUDE_DIRS" cflags-only-I)
  #message("${pkgname} include dirs: ${${_prefix}_INCLUDE_DIRS}")
  #set(${_prefix}_INCLUDE_DIRS ${${_prefix}_INCLUDE_DIRS} CACHE INTERNAL "")

  # Get the other cflags
  invoke_rospack(${pkgname} ${_prefix} temp cflags-only-other)
  _list_to_string(${_prefix}_CFLAGS_OTHER "${${_prefix}_temp}")
  #message("${pkgname} other cflags: ${${_prefix}_CFLAGS_OTHER}")
  set(${_prefix}_CFLAGS_OTHER ${${_prefix}_CFLAGS_OTHER} CACHE INTERNAL "")

  # Get the lib dirs
  invoke_rospack(${pkgname} ${_prefix} LIBRARY_DIRS libs-only-L)
  #message("${pkgname} library dirs: ${${_prefix}_LIBRARY_DIRS}")
  set(${_prefix}_LIBRARY_DIRS ${${_prefix}_LIBRARY_DIRS} CACHE INTERNAL "")

  # Get the libs
  invoke_rospack(${pkgname} ${_prefix} LIBRARIES libs-only-l)
  #
  # The following code removes duplicate libraries from the link line,
  # saving only the last one.
  #
  list(REVERSE ${_prefix}_LIBRARIES)
  list(REMOVE_DUPLICATES ${_prefix}_LIBRARIES)
  list(REVERSE ${_prefix}_LIBRARIES)

  # Get the other lflags
  invoke_rospack(${pkgname} ${_prefix} temp libs-only-other)
  _list_to_string(${_prefix}_LDFLAGS_OTHER "${${_prefix}_temp}")
  #message("${pkgname} other ldflags: ${${_prefix}_LDFLAGS_OTHER}")
  set(${_prefix}_LDFLAGS_OTHER ${${_prefix}_LDFLAGS_OTHER} CACHE INTERNAL "")
endmacro()

