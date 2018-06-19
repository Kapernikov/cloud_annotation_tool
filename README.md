# L-CAS 3D Point Cloud Annotation Tool #


* Maintainer status: maintained
* Author: Zhi Yan
* License: CC BY-NC-SA 4.0
* Dataset: [https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-3d-point-cloud-people-dataset/](https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-3d-point-cloud-people-dataset/)

The tool provides a semi-automatic labeling function, means the 3D point cloud data (loaded from the PCD file) is first clustered to provide candidates for labeling, each candidate being a point cluster. Then, the user annotating the data, can label each object by indicating candidate's ID, category, and visibility. A flowchart of this process is shown below.


## Compiling ##

### Prerequisites ###

#### New

* Qt 5.3: `sudo apt-get install qtbase5-dev qt5-qmake`
* VTK 6.2: `sudo apt-get install libvtk6-dev lib libvtk6-qt-dev`
* PCL 1.8.1: From Source


#### Old

* Qt 4.x: `sudo apt-get install libqt4-dev qt4-qmake`
* VTK 5.x: `sudo apt-get install libvtk5-dev`
* PCL 1.7: `sudo apt-get install libpcl-1.7-all-dev`

### Build script ###

* `mkdir build`
* `cd build`
* `cmake ..`
* `make`