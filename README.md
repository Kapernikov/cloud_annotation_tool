# adapted L-CAS 3D Point Cloud Annotation Tool / KPV modification Frank Dekervel #

* status: unfinished (does not work yet)

* forked from https://github.com/lesterlo/cloud_annotation_tool which was
  forked from https://github.com/yzrobot/cloud_annotation_tool

* modified the annotation process so that you can do point-by-point
  annotation (no clustering done in advance, you just get a brush to "paint"
  points)

* i had to modify the tool to use QVTKOpenGLWidget instead of QVTKWidget
  (because otherwise point picking does not work reliably)

## Running ##

if you get the following error: `vtkShaderProgram (0x556d0534a670): 0:1(10): error: GLSL 1.50 is not supported. Supported versions are: 1.10, 1.20, 1.30, 1.00 ES, 3.00 ES, 3.10 ES, and 3.20 ES`
then run the tool as follows: MESA_GL_VERSION_OVERRIDE=3.2 ./cloud_annotation_tool

## Compiling ##

### Prerequisites ###

#### New

* Qt 5.3: `sudo apt-get install qtbase5-dev qt5-qmake`
* VTK 8.1: please compile from master
* PCL MASTER branch: From Source git master branch of pcl

### Build script ###

* `mkdir build`
* `cd build`
* `cmake ..`
* `make`