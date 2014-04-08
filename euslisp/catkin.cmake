# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(euslisp)

find_package(catkin REQUIRED COMPONENTS rostest)

# build euslisp
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.eus
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of euslisp failed")
endif(_make_failed)

add_rostest(test/test-euslisp.test)
#check_for_display(disp)
#if(disp)
#  add_rostest(test/test-irtrobot.test)
#  add_rostest(test/test-irtviewer.test)
#endif(disp)

# build euslisp before catkin_package, since catkin_pakcage/INCLUDE_DIRS needs to be exists
catkin_package(
    DEPENDS opengl libjpeg libx11-dev libxext libpng12-dev
    CATKIN-DEPENDS # TODO
    INCLUDE_DIRS # jskeus/eus/include
    LIBRARIES # TODO
)

# install under EUSDIR/ARCHDIR ...
set(EUSDIR ${CATKIN_PACKAGE_SHARE_DESTINATION}/jskeus/eus)
if(${CMAKE_SYSTEM_NAME} MATCHES Linux)
  execute_process(COMMAND gcc -dumpmachine OUTPUT_VARIABLE GCC_MACHINE OUTPUT_STRIP_TRAILING_WHITESPACE)
  message("-- Set GCC_MACHINE to ${GCC_MACHINE}")
  if(${GCC_MACHINE} MATCHES x86_64-linux-gnu)
    set(ARCHDIR Linux64)
  elseif(${GCC_MACHINE} MATCHES i686-linux-gnu)
    set(ARCHDIR Linux)
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
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

# since euslisp is compile outside of cmake script, we need change rpath
foreach(executable ${executables})
  get_filename_component(filename ${executable} NAME)
  get_filename_component(rpath ${executable} PATH) # get path    .. eus/Linux64/bin
  get_filename_component(rpath ${rpath} PATH)      # get parent  .. eus/Linux64
  set(rpath "${rpath}/lib")                        # move to lib .. eus/Linux64/lib
  install(CODE "
    set(_rpath)
    file(STRINGS \"\$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/${ARCHDIR}/bin/${filename}\" _rpath REGEX ${rpath} LIMIT_COUNT 1)
    if(_rpath)
      file(RPATH_CHANGE
           FILE      \"\$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/${ARCHDIR}/bin/${filename}\"
           OLD_RPATH ${rpath}
           NEW_RPATH \"\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/${ARCHDIR}/lib\")
    endif()")
endforeach()

install(CODE "
  message(\"-- CreateDirectory: \"$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/bin\"\")
  file(MAKE_DIRECTORY \"\$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin\")
")
foreach(executable ${executables})
  get_filename_component(filename ${executable} NAME)
  install(CODE "
    message(\"-- CreateLink: \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/${filename} -> \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/${ARCHDIR}/bin/${filename}\")
    execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink ../${EUSDIR}/${ARCHDIR}/bin/${filename} \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/${filename})
")
endforeach()

# libraries
install(DIRECTORY jskeus/eus/${ARCHDIR}/lib/
  DESTINATION ${EUSDIR}/${ARCHDIR}/lib
  USE_SOURCE_PERMISSIONS
)

# objs
install(DIRECTORY jskeus/eus/${ARCHDIR}/obj/
  DESTINATION ${EUSDIR}/${ARCHDIR}/obj
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.l" PATTERN "*.so"  PATTERN ".svn" EXCLUDE
)

# includes
install(CODE "
  message(\"-- CreateLink: \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/include -> \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/lisp/c\")
  execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink lisp/c \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/include)
")

# lib
install(DIRECTORY jskeus/eus/lib/
  DESTINATION ${EUSDIR}/lib
  FILES_MATCHING PATTERN "*.l" PATTERN ".svn" EXCLUDE
)

# lisp
install(DIRECTORY jskeus/eus/lisp/
  DESTINATION ${EUSDIR}/lisp
  FILES_MATCHING PATTERN "*" PATTERN ".svn" EXCLUDE
)

# models
install(DIRECTORY jskeus/eus/models/
  DESTINATION ${EUSDIR}/models
  FILES_MATCHING PATTERN "*.l" PATTERN "*.jpg" PATTERN "*.png" PATTERN "*.ppm" PATTERN ".svn" EXCLUDE
)

# img
install(DIRECTORY jskeus/eus/img/
  DESTINATION ${EUSDIR}/img
  FILES_MATCHING PATTERN "*.l" PATTERN "*.jpg" PATTERN "*.png" PATTERN "*.ppm" PATTERN ".svn" EXCLUDE
)

# manuals
install(FILES
  jskeus/manual.pdf
  jskeus/jmanual.pdf
  jskeus/bashrc.eus
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/jskeus
)
install(FILES jskeus/eus/doc/latex/manual.pdf DESTINATION ${EUSDIR}/doc/latex)
install(FILES jskeus/eus/doc/jlatex/jmanual.pdf DESTINATION ${EUSDIR}/doc/jlatex)

# irteus
install(DIRECTORY jskeus/irteus/
  DESTINATION ${EUSDIR}/../irteus
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*" PATTERN ".svn" EXCLUDE
)
# includes
install(CODE "
  message(\"-- CreateLink: \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/irteus -> \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/../irteus\")
  execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink ../irteus \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/irteus)
")

# could not found a way to set custom install directory for installed configuration file
install(CODE "
   message(\"-- Force change Cflags of \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/euslisp.pc \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/\${PROJECT_NAME}Config.cmake\")
   execute_process(COMMAND sed -i s@\${CMAKE_INSTALL_PREFIX}/include@\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/include@ \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/euslisp.pc \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/${PROJECT_NAME}Config.cmake)
   ")



install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
