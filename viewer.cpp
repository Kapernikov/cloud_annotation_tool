#include "viewer.h"
#include "build/ui_viewer.h"
#include "annotatorinteractor.h"
#include "vtkGenericOpenGLRenderWindow.h"

CloudViewer::CloudViewer ( QWidget *parent ) :
    QMainWindow ( parent ),
    ui ( new Ui::CloudViewer )
{
    ui->setupUi ( this );
    this->setWindowIcon ( QIcon ( ":/images/lcas_logo.png" ) );
    this->setWindowTitle ( "L-CAS Cloud Annotation Tool" );
    load_file_path = std::getenv ( "HOME" );

    auto_next = false;
    removing_outliers = false;
    downsampling = false;
    removing_planes = false;


    vtkSmartPointer<AnnotatorInteractor> interactorStyle = vtkSmartPointer<AnnotatorInteractor>::New();

    // Set up the QVTK window.
    auto renderer = vtkSmartPointer <vtkRenderer>::New();
    _renderWindow = vtkSmartPointer <vtkGenericOpenGLRenderWindow>::New();
    _renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, _renderWindow, "PCL Viewer", false));
    ui->qvtkWidget->SetRenderWindow ( viewer->getRenderWindow() );
    interactorStyle->Initialize();

    // Create the interactor style
    auto rens = viewer->getRendererCollection();
    interactorStyle->setRendererCollection ( rens );
    interactorStyle->setCloudActorMap ( viewer->getCloudActorMap() );
    interactorStyle->setShapeActorMap ( viewer->getShapeActorMap() );
    //interactorStyle->setRenderWindow(viewer->getRenderWindow());
    interactorStyle->UseTimersOn();
    interactorStyle->registerPaintingCallback ( [this] ( double x, double y, double z , long pointid) {
        painted ( x,y,z, pointid );
    } );
    //interactorStyle->setUseVbos(viewer->get);


    viewer->setupInteractor ( ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow(), interactorStyle );
    //viewer->setupInteractor ( ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());

    // Set rend erer window in case no interactor is created


    ui->qvtkWidget->update();

    // Connect UI and their functions.
    connect ( ui->pushButton_load, &QPushButton::clicked, this, &CloudViewer::loadButtonClicked );
    connect ( ui->pushButton_label, &QPushButton::clicked, this, &CloudViewer::labelButtonClicked );
    connect ( ui->pushButton_reload, &QPushButton::clicked, this, &CloudViewer::reloadButtonClicked );
    connect ( ui->checkBox_next, &QCheckBox::stateChanged, this, &CloudViewer::nextBoxChecked );
    connect ( ui->listWidget_files, &QListWidget::itemSelectionChanged, this, &CloudViewer::fileItemChanged );
}

CloudViewer::~CloudViewer()
{
    delete ui;
}

void CloudViewer::loadButtonClicked()
{
    // Load pcd files.
    QStringList files = QFileDialog::getOpenFileNames ( this, tr ( "Select one or more files to open" ), load_file_path, tr ( "Point cloud data(*.pcd)" ) );
    if ( files.isEmpty() ) {
        return;
    }
    load_file_path = QFileInfo ( files[0] ).absolutePath();

    // Show file list.
    for ( QStringList::Iterator it = files.begin(); it != files.end(); ++it ) {
        QList<QListWidgetItem*> items = ui->listWidget_files->findItems ( *it, Qt::MatchExactly );
        if ( items.size() == 0 ) {
            ui->listWidget_files->addItem ( *it );
        }
    }
    if ( ui->listWidget_files->currentRow() == -1 ) {
        ui->listWidget_files->setCurrentRow ( 0 );
    }
}

void CloudViewer::labelButtonClicked()
{
    if ( ui->listWidget_files->currentRow() == -1 ) {
        return;
    }

    if ( QDir ( current_label_path ).exists() == false ) {
        QDir().mkpath ( current_label_path );
    }

    std::string string_to, line_to, line_in;
    for ( std::vector<Feature>::iterator it = features.begin(); it != features.end(); ++it ) {
        if ( boost::to_string ( it->id ) == ui->lineEdit_object_id->text().toStdString() ) {
            bool new_label = true, change_label = false;
            line_to = boost::to_string ( it->centroid[0] )+" "+boost::to_string ( it->centroid[1] )+" "+boost::to_string ( it->centroid[2] )+" "+
                      boost::to_string ( it->min[0] )+" "+boost::to_string ( it->min[1] )+" "+boost::to_string ( it->min[2] )+" "+
                      boost::to_string ( it->max[0] )+" "+boost::to_string ( it->max[1] )+" "+boost::to_string ( it->max[2] );
            label_file.open ( ( current_label_path+"/"+QFileInfo ( ui->listWidget_files->currentItem()->text() ).completeBaseName()+".txt" ).toStdString().c_str(), std::fstream::in );
            while ( std::getline ( label_file, line_in ) ) {
                if ( line_in.substr ( line_in.find ( " " )+1, line_in.length()-line_in.find ( " " )-3 ).compare ( line_to ) == 0 ) {
                    new_label = false;
                    if ( ui->comboBox_class->currentText().toStdString().compare ( "dontcare" ) == 0 ) {
                        viewer->removeText3D ( "labeled_text_"+boost::to_string ( it->centroid[0] ) );
                        viewer->removeShape ( "labeled_box_"+boost::to_string ( it->centroid[0] ) );
                        ui->label_show->setText ( "<font color=\"blue\">Label removed.</font>" );
                        continue;
                    } else {
                        if ( ui->comboBox_class->currentText().toStdString().compare ( line_in.substr ( 0, line_in.find ( " " ) ) ) == 0 &&
                                boost::to_string ( ui->comboBox_visibility->currentIndex() ).compare ( line_in.substr ( line_in.length()-1 ) ) == 0 ) {
                            ui->label_show->setText ( "<font color=\"blue\">Do nothing.</font>" );
                        } else {
                            change_label = true;
                            viewer->removeText3D ( "labeled_text_"+boost::to_string ( it->centroid[0] ) );
                            viewer->removeShape ( "labeled_box_"+boost::to_string ( it->centroid[0] ) );
                            ui->label_show->setText ( "<font color=\"blue\">Label changed.</font>" );
                            continue;
                        }
                    }
                }
                string_to += line_in+"\n";
            }
            if ( new_label || change_label ) {
                if ( ui->comboBox_class->currentText().toStdString().compare ( "dontcare" ) == 0 ) {
                    ui->label_show->setText ( "<font color=\"blue\">Do nothing.</font>" );
                } else {
                    string_to += ui->comboBox_class->currentText().toStdString()+" "+line_to+" "+
                                 boost::to_string ( ui->comboBox_visibility->currentIndex() )+"\n";
                    double r, g, b;
                    if ( ui->comboBox_class->currentText().toStdString().compare ( "pedestrian" ) == 0 ) {
                        r=1;
                        g=0;
                        b=0;
                    }
                    if ( ui->comboBox_class->currentText().toStdString().compare ( "group" ) == 0 )      {
                        r=0;
                        g=1;
                        b=0;
                    }
                    if ( ui->comboBox_class->currentText().toStdString().compare ( "wheelchair" ) == 0 ) {
                        r=0;
                        g=0;
                        b=1;
                    }
                    if ( ui->comboBox_class->currentText().toStdString().compare ( "cyclist" ) == 0 )    {
                        r=1;
                        g=1;
                        b=0;
                    }
                    if ( ui->comboBox_class->currentText().toStdString().compare ( "car" ) == 0 )        {
                        r=0;
                        g=1;
                        b=1;
                    }
                    pcl::PointXYZ pos ( it->centroid[0], it->centroid[1], it->max[2] );
                    viewer->addText3D ( ui->comboBox_class->currentText().toStdString(), pos, 0.2, r, g, b, "labeled_text_"+boost::to_string ( it->centroid[0] ) );
                    viewer->addCube ( it->min[0], it->max[0], it->min[1], it->max[1], it->min[2], it->max[2], r, g, b, "labeled_box_"+boost::to_string ( it->centroid[0] ) );
                    viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "labeled_box_"+boost::to_string ( it->centroid[0] ) );
                    viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "labeled_box_"+boost::to_string ( it->centroid[0] ) );
                    if ( change_label == false ) {
                        ui->label_show->setText ( "<font color=\"blue\">Label added.</font>" );
                    }
                }
            }
            label_file.close();
            label_file.open ( ( current_label_path+"/"+QFileInfo ( ui->listWidget_files->currentItem()->text() ).completeBaseName()+".txt" ).toStdString().c_str(), std::fstream::out | std::fstream::trunc );
            label_file << string_to;
            label_file.close();
            ui->qvtkWidget->update();
            if ( auto_next ) {
                ui->listWidget_files->setCurrentRow ( ui->listWidget_files->currentRow()+1 );
            }
            return;
        }
    }
    ui->label_show->setText ( "<font color=\"red\">Unknown ID.</font>" );
}

void CloudViewer::reloadButtonClicked()
{
    if ( ui->listWidget_files->currentRow() != -1 ) {
        fileItemChanged();
    }
}

void CloudViewer::nextBoxChecked()
{
    auto_next = ( ui->checkBox_next->isChecked() ) ? true : false;
}

void CloudViewer::fileItemChanged()
{
    loadFile ( ui->listWidget_files->currentItem()->text().toStdString() );

    file_labeled = false;
    current_label_path = QFileInfo ( ui->listWidget_files->currentItem()->text() ).absolutePath()+"/label";
    if ( QDir ( current_label_path ).exists() ) {
        QDirIterator dir_it ( current_label_path, QDirIterator::Subdirectories );
        QString file;
        while ( dir_it.hasNext() ) {
            file = dir_it.next();
            if ( QFileInfo ( file ).fileName().toStdString().find ( '~' ) == std::string::npos &&
                    QFileInfo ( file ).completeBaseName() == QFileInfo ( ui->listWidget_files->currentItem()->text() ).completeBaseName() ) {
                label_file.open ( ( current_label_path+"/"+QFileInfo ( ui->listWidget_files->currentItem()->text() ).completeBaseName()+".txt" ).toStdString().c_str(), std::fstream::in );
                labels.clear();
                std::string line;
                while ( std::getline ( label_file, line ) ) {
                    std::vector<std::string> params;
                    boost::split ( params, line, boost::is_any_of ( " " ) );
                    labels.push_back ( params );
                    pcl::PointXYZ pos ( atof ( params[1].c_str() ), atof ( params[2].c_str() ), atof ( params[9].c_str() ) );
                    double r, g, b;
                    if ( params[0].compare ( "pedestrian" ) == 0 ) {
                        r=1;
                        g=0;
                        b=0;
                    }
                    if ( params[0].compare ( "group" ) == 0 )      {
                        r=0;
                        g=1;
                        b=0;
                    }
                    if ( params[0].compare ( "wheelchair" ) == 0 ) {
                        r=0;
                        g=0;
                        b=1;
                    }
                    if ( params[0].compare ( "cyclist" ) == 0 )    {
                        r=1;
                        g=1;
                        b=0;
                    }
                    if ( params[0].compare ( "car" ) == 0 )        {
                        r=0;
                        g=1;
                        b=1;
                    }
                    viewer->addText3D ( params[0], pos, 0.2, r, g, b, "labeled_text_"+params[1] );
                    viewer->addCube ( atof ( params[4].c_str() ), atof ( params[7].c_str() ),
                                      atof ( params[5].c_str() ), atof ( params[8].c_str() ),
                                      atof ( params[6].c_str() ), atof ( params[9].c_str() ),
                                      r, g, b, "labeled_box_"+boost::to_string ( params[1] ) );
                    viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "labeled_box_"+boost::to_string ( params[1] ) );
                    viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "labeled_box_"+boost::to_string ( params[1] ) );
                    ui->qvtkWidget->update();
                }
                label_file.close();
                file_labeled = true;
                ui->label_show->setText ( "<font color=\"blue\">File labeled.</font>" );
                break;
            }
        }
    }
    if ( file_labeled == false ) {
        ui->label_show->setText ( "<font color=\"blue\">File without labels.</font>" );
    }
}

void CloudViewer::painted ( double x, double y, double z , const long pointid)
{
    //std::cout << "callback " << x << " " << y << " " << z << " " << pointid << std::endl;
    movePointToAnn(x, y, z, pointid);

}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudViewer::colorize(pcl::PointCloud<pcl::PointXYZ> &source, int r, int g, int b)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_p;
    new_p.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    for (auto pt: source.points) {
        pcl::PointXYZRGBA zpt;
        zpt.x = pt.x;
        zpt.y = pt.y;
        zpt.z = pt.z;
        zpt.r = r;
        zpt.g = g;
        zpt.b = b;
        zpt.a = 255;
        new_p->points.push_back(zpt);
    }
    return new_p;
}

void CloudViewer::movePointToAnn ( double x, double y, double z , long pointid)
{
    const int K=30;
    const float radius=0.1; // 10cm
    std::vector<int> indices ( K );
    std::vector<float> distances ( K );
    pcl::PointXYZRGBA xyz;
    xyz.x = x; xyz.y =  y; xyz.z = z;
    if ( kdtree.radiusSearch ( xyz, radius, indices, distances, K ) > 0 ) {
        for (auto &idx: indices) {
            auto &pt = cloud->points.at(idx);
            //std::cout << "found point : " << (int)(pt.r) << " " << (int)(pt.g) << " " << (int)(pt.b) << " " << indices.at(0) << " " << pointid <<  std::endl;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            pt.a = 255;
        }
    }
    viewer->updatePointCloud(cloud, "cloud");
    _renderWindow->Render();
    ui->qvtkWidget->repaint();
    

}

void CloudViewer::loadFile ( std::string file_name )
{
    cloud.reset ( new pcl::PointCloud<pcl::PointXYZRGBA> );
    cloud_ann.reset(new pcl::PointCloud<pcl::PointXYZRGBA> );

    pcl::PointCloud<pcl::PointXYZ> xyz_cloud, cloud_tmp;

    // Load point cloud.
    if ( pcl::io::loadPCDFile ( file_name, cloud_tmp ) != 0 ) {
        PCL_ERROR ( "Error reading point cloud %s\n", file_name.c_str() );
        return;
    }

    if ( cloud_tmp.is_dense ) {
        pcl::copyPointCloud ( cloud_tmp, xyz_cloud );
        cloud = colorize(cloud_tmp, 255,255,255);
    } else {
        PCL_WARN ( "Cloud is not dense! Non finite points will be removed\n" );
        std::vector<int> vec;
        pcl::removeNaNFromPointCloud ( cloud_tmp, xyz_cloud, vec );
        cloud = colorize(xyz_cloud, 255,255,255);
    }

    kdtree = pcl::KdTreeFLANN<pcl::PointXYZRGBA>();
    kdtree.setInputCloud ( cloud );

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    features.clear();

    pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud);

    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
    //col ( cloud, 255, 255, 255 );

    viewer->addPointCloud ( cloud, rgba, "cloud" );
    //viewer->addCoordinateSystem();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid ( *cloud, centroid );
/*
    viewer->setCameraPosition (
        centroid[0],
        centroid[1] + 2,
        centroid[2] + 100,

        centroid[0],
        centroid[1],
        centroid[2],

        0,
        1,
        0,
        0
    );
*/
    std::cout << "centroid: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;

    ui->qvtkWidget->update();
}

