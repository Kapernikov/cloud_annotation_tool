cmake_minimum_required (VERSION 2.8.11 FATAL_ERROR)

project(cloud_annotation_tool)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package (Qt5 REQUIRED COMPONENTS Widgets Core)
find_package (VTK REQUIRED)
find_package (PCL 1.8 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set (project_SOURCES   main.cpp viewer.cpp)
set (project_HEADERS   viewer.h)
set (project_FORMS     viewer.ui)
set (project_RESOURCES resources.qrc)
set (VTK_LIBRARIES     vtkRendering vtkGraphics vtkHybrid QVTK)


ADD_EXECUTABLE (${PROJECT_NAME} ${project_SOURCES})
TARGET_LINK_LIBRARIES (${PROJECT_NAME} ${PCL_LIBRARIES})
qt5_use_modules(${PROJECT_NAME} Widgets Core)
