# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(euslisp)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_package(
    DEPENDS opengl libjpeg libx11-dev libxext libpng12
    CATKIN-DEPENDS # TODO
    INCLUDE_DIRS # TODO need to add ${EUSDIR}/include and other euslisp compile flags
    LIBRARIES # TODO
)


# build euslisp
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.eus
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of euslisp failed")
endif(_make_failed)

add_rostest(test/test-euslisp.launch)
#check_for_display(disp)
#if(disp)
#  add_rostest(test/test-irtrobot.launch)
#endif(disp)

# install under EUSDIR/ARCHDIR ...
set(EUSDIR ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(${CMAKE_SYSTEM_NAME} MATCHES Linux)
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
      ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
    set(ARCHDIR Linux64)
  else()
    set(ARCHDIR Linux)
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set(ARCHDIR Darwin)
elseif(${CMAKE_SYSTEM_NAME} MATCHES Cygwin)
  set(ARCHDIR Cygwin)
else()
  set(ARCHDIR Generic)
endif()
message("-- Set EUSDIR  to ${CMAKE_INSTALL_PREFIX}/${EUSDIR}")
message("-- Set ARCHDIR to ${ARCHDIR}")

# for all executable files under ${PROJECT_SOURCE_DIR}/jskeus/eus/Linux64/bin/
file(GLOB executables "${PROJECT_SOURCE_DIR}/jskeus/eus/${ARCHDIR}/bin/*")
install(PROGRAMS ${executables} # PROGRAM command installs file and chmod u+x
        DESTINATION ${EUSDIR}/${ARCHDIR}/bin/
)
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} original distination


# strip rpath, since euslisp is compile outside of cmake script, we need post install script to change rpath
file(WRITE ${CMAKE_BINARY_DIR}/post_install.cmake
  "# strip rpath, since euslisp is compile outside of cmake script, we need post install script to change rpath")
foreach(executable ${executables})
  get_filename_component(target ${executable} NAME)
  set(exe \${DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/${ARCHDIR}/bin/${target})
  get_filename_component(rpath ${executable} PATH) # get path   .. eus/Linux64/bin
  get_filename_component(rpath ${rpath} PATH)      # get parent .. eus/Linux64/
  set(rpath "${rpath}/lib")                        # move to lib . eus/Linux64/lib
  file(APPEND ${CMAKE_BINARY_DIR}/post_install.cmake "
    file(RPATH_CHECK FILE \"${exe}\" RPATH ${rpath}) ## this removes target file, so we need recopy them
    if(NOT EXISTS \"${exe}\")
      file(COPY ${executable} DESTINATION  \${DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/${ARCHDIR}/bin)
    else()
      file(RPATH_CHANGE
           FILE      ${exe}
           OLD_RPATH ${rpath}
           NEW_RPATH \${DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/${ARCHDIR}/lib)
    endif()
    file(MAKE_DIRECTORY \${DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin)
    message(\"-- create_symlink ${exe} \${DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/${target}\")
    execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink ${exe} \${DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/${target})
  ")
endforeach()
install(SCRIPT ${CMAKE_BINARY_DIR}/post_install.cmake)

# libraries
install(DIRECTORY jskeus/eus/${ARCHDIR}/lib/
  DESTINATION ${EUSDIR}/${ARCHDIR}/lib
)

# includes
install(DIRECTORY jskeus/eus/lisp/c/
  DESTINATION ${EUSDIR}/include
  FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE
)

# lib
install(DIRECTORY jskeus/eus/lib/
  DESTINATION ${EUSDIR}/lib
  FILES_MATCHING PATTERN "*.l" PATTERN ".svn" EXCLUDE
)

# lisp
install(DIRECTORY jskeus/eus/lisp/
  DESTINATION ${EUSDIR}/lisp
  FILES_MATCHING PATTERN "*.l" PATTERN ".svn" EXCLUDE
)

# models
install(DIRECTORY jskeus/eus/models/
  DESTINATION ${EUSDIR}/models
  FILES_MATCHING PATTERN "*.l" PATTERN "*.jpg" PATTERN "*.png" PATTERN "*.ppm" PATTERN ".svn" EXCLUDE
)

# manuals
install(FILES
  jskeus/manual.pdf
  jskeus/jmanual.pdf
  jskeus/bashrc.eus
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# dummy test target
add_custom_target(test COMMAND echo "DUMMY test target")
add_custom_target(test-results COMMAND echo "DUMMY test-results target")
