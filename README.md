# adapted L-CAS 3D Point Cloud Annotation Tool / KPV modification Frank Dekervel #

![Screenshot](screenshot.png?raw=true "Screenshot")


* forked from https://github.com/lesterlo/cloud_annotation_tool which was
  forked from https://github.com/yzrobot/cloud_annotation_tool

* modified the annotation process so that you can do point-by-point
  annotation (no clustering done in advance, you just get a brush to "paint"
  points)

* i had to modify the tool to use QVTKOpenGLWidget instead of QVTKWidget
  (because otherwise point picking does not work reliably)

## Building

Because this project needs VTK8 and pcl master (1.8.1 is NOT new enough), it can be a hassle to
set up. The easiest way to set up the project is building a snap:

* cd snap
* snapcraft
* snap install --devmode *.snap
* cloud-annotator-tool

Alternatively, if you want to make changes and you don't have a nvidia card,
running in docker might also be suitable:

* build the docker image:  `docker build -t vtk8_pclmaster docker/`
* then disable xhost auth `xhost +` (or use x11-docker)
* then run a container: `docker run -it --rm -v $PWD:$PWD -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix vtk8_pclmaster /bin/bash`
* inside the container, go back to the folder where you checked out the source and do `cd cloud_annotator_tool; mkdir build; cd build; cmake -DCMAKE_BUILD_TYPE=Release ..; make -j3`
* now you can start the cloud annotator tool


## Remaining issues ##

* sometimes after panning/zooming/flying the cloud "looses" a number of points. looks like a VTK
  bug. Save your data and reopen the app, no data is lost.

* clipping settings do not work properly yet

* vtk8 glsl issue, use MESA_GL_VERSION_OVERRIDE=3.2 to work around (see below)

## Running ##

if you get the following error: `vtkShaderProgram (0x556d0534a670): 0:1(10): error: GLSL 1.50 is not supported. Supported versions are: 1.10, 1.20, 1.30, 1.00 ES, 3.00 ES, 3.10 ES, and 3.20 ES`
then run the tool as follows: MESA_GL_VERSION_OVERRIDE=3.2 ./cloud_annotation_tool

* open a point cloud
* create a new segment by filling in object id and object class and clicking add
* click on the pointcloud and zoom it to the right place (use the "f" key to fly to a point)
* to add points to the cluster, move your mouse while holding the "p" key. to remove points from the cluster, move your mouse by holding the "e" key
* the GUI saves a json file every time you switch to another cluster (so switch to another cluster before quitting). the json file is also load back when you open the pcd file

### Prerequisites ###

#### New

* Qt 5.3: `sudo apt-get install qtbase5-dev qt5-qmake`
* VTK 8.1: please compile from master
* PCL MASTER branch: From Source git master branch of pcl
