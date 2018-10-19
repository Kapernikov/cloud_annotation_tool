FROM ubuntu:18.04
MAINTAINER frank@kapernikov.com
RUN apt-get update && apt-get install -y build-essential cmake qttools5-dev-tools libqt5opengl5-dev libqt5x11extras5-dev qtbase5-dev libqt5opengl5-dev python3-dev python3-pip git wget \
	unzip yasm libpng-dev libavformat-dev libtiff-dev libswscale-dev llvm-dev libeigen3-dev libflann-dev libqhull-dev libopenni-dev libboost-dev \
	libboost-system-dev libboost-filesystem-dev libboost-thread-dev libboost-date-time-dev libboost-iostreams-dev
RUN mkdir -p /opt/source && cd /opt/source && git clone https://github.com/Kitware/VTK.git && cd VTK && git checkout v8.1.1 && cd .. && git clone https://github.com/PointCloudLibrary/pcl
WORKDIR /opt/source
RUN apt-get -y install libxt-dev libqt5gui5 libxml2-dev mpi-default-dev libqt5webkit5-dev qttools5-dev libopenni-dev libopenni2-dev 
RUN cd VTK && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DVTK_Group_Qt=ON -DVTK_Group_Rendering=ON -DVTK_Group_Views=ON -DVTK_Group_MPI=ON -DVTK_PYTHON_VERSION=3 -DVTK_WRAP_PYTHON=ON .. \
    && make -j6 install
RUN apt-get -y install libusb-dev libusb-1.0-0-dev
RUN cd pcl && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_features=ON -DBUILD_filters=ON -DBUILD_geometry=ON -DBUILD_search=ON -DBUILD_tools=ON -DBUILD_visualization=ON -DBUILD_tracking=ON -DBUILD_apps=ON  -DPCL_QT_VERSION=5 -DBUILD_apps_point_cloud_editor=ON  .. \
    && make -j6 install



