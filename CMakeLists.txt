# Reference: http://qt-project.org/doc/qt-5.0/qtdoc/cmake-manual.html

cmake_minimum_required(VERSION 3.0)

# Add folder where are supportive functions
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include Qt basic functions
include(QtCommon)

# Basic information about project

project(example VERSION 1.0)

find_package(CGAL REQUIRED COMPONENTS Core)
include( ${CGAL_USE_FILE} )

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

SET(EXTRA_CXX_COMPILE_FLAGS "-std=c++11 -I./src -I./test -I/opt/local/include -I/Users/dillon/CppWorkspace/gca/src -O2")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_COMPILE_FLAGS}")

SET(CXX_OCL_LINK_FLAGS "-L/Users/dillon/CppWorkspace/gca/ -locl -locl.11.10.66")
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${CXX_OCL_LINK_FLAGS}")

# Set PROJECT_VERSION_PATCH and PROJECT_VERSION_TWEAK to 0 if not present, needed by add_project_meta
fix_project_version()

# Set additional project information
set(COMPANY "Example")
set(COPYRIGHT "Copyright (c) 2014 Vincent Lee. All rights reserved.")
set(IDENTIFIER "com.example.Example")

set(SOURCE_FILES
    src/main.cc
    src/mainwindow.cc
)

add_project_meta(META_FILES_TO_INCLUDE)

set(RESOURCE_FILES example.qrc)

find_package(Qt5Widgets REQUIRED)

# source_group("UI Files" FILES ${UI_FILES})

add_library(partslice src/part_slicing.cpp)
target_link_libraries(partslice ${VTK_LIBRARIES} ${CGAL_LIBRARIES} utils backend gprocess geometry gcode gca)

add_executable(${PROJECT_NAME} ${OS_BUNDLE} # Expands to WIN32 or MACOS_BUNDLE depending on OS
    ${SOURCE_FILES} ${META_FILES_TO_INCLUDE} ${RESOURCE_FILES}
)
target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES} ${CGAL_LIBRARIES} utils backend gprocess geometry gcode gca partslice)

qt5_use_modules(${PROJECT_NAME} Widgets)

add_executable(all-tests test/coplanar_tests.cpp)
target_link_libraries(all-tests ${VTK_LIBRARIES} ${CGAL_LIBRARIES} utils backend gprocess geometry gcode gca partslice)


# Build commands
# mkdir yet_again_qt_vtk
# cd yet_again_qt_vtk/
# git clone https://github.com/euler0/mini-cmake-qt.git
# cd mini-cmake-qt/
# cmake -DCMAKE_PREFIX_PATH=/Users/dillon/Qt/5.5/clang_64/lib/cmake/Qt5Widgets/ -DVTK_DIR:PATH=/Users/dillon/Downloads/VTK-build-qt-gui/ .
