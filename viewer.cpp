#include "viewer.h"
#include "build/ui_viewer.h"
#include "annotatorinteractor.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "qmessagebox.h"
#include <cmath>
#include "qinputdialog.h"

#include <pcl/visualization/common/actor_map.h>


void CloudViewer::colorizeCloud(pcl::PointCloud<pcl::PointXYZRGBA> &c, char r, char g, char b, char a) {
    for (auto &pt: c.points) {
        pt.r = r;
        pt.g = g;
        pt.b = b;
        pt.a = a;
    }
    for (auto &it: segments) {
        auto &k = it.second;
        for (auto idx: k) {
            auto &pt = c.points.at(idx);
            pt.r = pasCluster_R;
            pt.g = pasCluster_G;
            pt.b = pasCluster_B;
        }
    }

}

void CloudViewer::saveJson()
{
    json j = segments;
    std::string fn =  ui->listWidget_files->currentItem()->text().toStdString();
    fn += "_labels.json";
    std::ofstream out(fn);
    out << j;
    out.close();
}

void CloudViewer::resetClippingSpinboxes()
{
    std::vector<pcl::visualization::Camera> cams;
    viewer->getCameras(cams);
    auto fc = cams.at(0);

    pcl::PointXYZRGBA minPt, maxPt;
    pcl::getMinMax3D (*cloud, minPt, maxPt);
    ui->spMinX->setValue(minPt.x);
    ui->spMaxX->setValue(maxPt.x);
    ui->spMinY->setValue(minPt.y);
    ui->spMaxY->setValue(maxPt.y);
    ui->spMinZ->setValue(minPt.z);
    ui->spMaxZ->setValue(maxPt.z);
    ui->spMinD->setValue(fc.clip[0]);
    ui->spMaxD->setValue(fc.clip[1]);
}

void CloudViewer::updateCameraClipping()
{
    if (ui->chkEnableClipping->isChecked()) {
        interactorStyle->SetAutoAdjustCameraClippingRange (false);
        _renderWindow->GetRenderers()->GetFirstRenderer()->ResetCameraClippingRange(
                    ui->spMinX->value(),ui->spMaxX->value(),
                    ui->spMinY->value(),ui->spMaxY->value(),
                    ui->spMinZ->value(),ui->spMaxZ->value()
        );
        viewer->setCameraClipDistances(ui->spMinD->value(),ui->spMaxD->value());
        _renderWindow->Render();
    } else {
        interactorStyle->SetAutoAdjustCameraClippingRange (true);
        _renderWindow->GetRenderers()->GetFirstRenderer()->ResetCameraClippingRange();
        _renderWindow->Render();
    }
}

void CloudViewer::confirmDeleteCurrentCluster()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Test", QString::fromStdString("Are you sure you want to delete " +  currentCluster.objectid + "/" + currentCluster.oclass + "?"),
                                  QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        auto cc = currentCluster;
        currentCluster.objectid = "";
        currentCluster.oclass = "";
        segments.erase(cc);
        ui->tblClusters->removeRow(ui->tblClusters->currentRow()); // this triggers saving again, so we must first erase the key of currentCluster
    }
}

void CloudViewer::renameCurrentCluster()
{
    bool ok;
    QString oldId = QString::fromStdString(currentCluster.objectid);
    QString newId = QInputDialog::getText(this,"New ID","Enter new object ID", QLineEdit::Normal,
                                          oldId,&ok);
    if (ok & (oldId != newId)) {
        auto r = ui->tblClusters->selectionModel()->selectedRows();
        if (r.size() > 0) {
            int sel_row = r.at(0).row();
            auto cc = currentCluster;
            ui->tblClusters->setItem( sel_row ,
                                      1, new QTableWidgetItem(newId));
            currentCluster.objectid = newId.toStdString();
            saveCurrentCluster();
            segments.erase(cc);
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudViewer::getCluster(ClusterKey &key)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGBA>());
    if (segments.count(key) == 0) {
        return p;
    }
    for (auto idx: segments[key]) {
        p->points.push_back(cloud->points.at(idx));
    }
    return p;
}

void CloudViewer::flyToCluster(ClusterKey &key)
{
    auto c = getCluster(key);
    if (c->points.size() == 0) {
        return;
    }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid ( *c, centroid );
    interactorStyle->GetInteractor()->FlyTo(interactorStyle->GetCurrentRenderer(), centroid[0], centroid[1], centroid[2]);
    _renderWindow->Render();
}

void CloudViewer::flyToSelectedCluster()
{
    flyToCluster(currentCluster);
}

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


    interactorStyle = vtkSmartPointer<AnnotatorInteractor>::New();

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
    interactorStyle->registerPaintingCallback ( [this] ( double x, double y, double z , long pointid, bool painting) {
        painted ( x,y,z, pointid, painting );
    } );


    viewer->setupInteractor ( ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow(), interactorStyle );


    ui->qvtkWidget->update();

    // Connect UI and their functions.
    connect ( ui->pushButton_load, &QPushButton::clicked, this, &CloudViewer::loadButtonClicked );
    connect ( ui->btnStartStop, &QPushButton::clicked, this, &CloudViewer::labelButtonClicked );
    connect ( ui->listWidget_files, &QListWidget::itemSelectionChanged, this, &CloudViewer::fileItemChanged );

    context.reset(new QMenu(this));
    connect(context->addAction("Fly to"), &QAction::triggered, [this]() {this->flyToSelectedCluster();});
    connect(context->addAction("Refresh"), &QAction::triggered, [this]() {this->viewer->updatePointCloud(this->cloud, "cloud");});
    connect(context->addAction("Rename cluster"), &QAction::triggered, [this]() { this->renameCurrentCluster();});
    connect(context->addAction("Delete cluster"), &QAction::triggered, [this]() { this->confirmDeleteCurrentCluster();});
    ui->tblClusters->setContextMenuPolicy(Qt::CustomContextMenu);
    connect (ui->tblClusters, &QTableWidget::customContextMenuRequested, [this](const QPoint& p) { this->context->exec(this->ui->tblClusters->mapToGlobal(p));});

    auto func = [this](double d) {if (this->ui->chkEnableClipping->isChecked()) { updateCameraClipping(); }};
    connect( ui->chkEnableClipping, &QCheckBox::toggled, [this]() {
        bool b = this->ui->chkEnableClipping->isChecked();
        this->ui->spMinX->setEnabled(b);
        this->ui->spMaxX->setEnabled(b);
        this->ui->spMinY->setEnabled(b);
        this->ui->spMaxY->setEnabled(b);
        this->ui->spMinZ->setEnabled(b);
        this->ui->spMaxZ->setEnabled(b);
        this->ui->spMinD->setEnabled(b);
        this->ui->spMaxD->setEnabled(b);
        this->updateCameraClipping();
    });
    connect( ui->spMinX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), func);
    connect( ui->spMaxX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), func);
    connect( ui->spMinY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), func);
    connect( ui->spMaxY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), func);
    connect( ui->spMinZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), func);
    connect( ui->spMaxZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), func);
    connect( ui->spMinD, QOverload<double>::of(&QDoubleSpinBox::valueChanged), func);
    connect( ui->spMaxD, QOverload<double>::of(&QDoubleSpinBox::valueChanged), func);

}

bool CloudViewer::hasSegment(std::string objectClass, std::string objectId)
{
    ClusterKey k;
    k.oclass = objectClass;
    k.objectid = objectId;
    if (segments.count(k) > 0) {
        return true;
    } else {
        return false;
    }
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
    QStringList labels;
    labels << tr("Class") << tr("Id") << tr("#Points");
    ui->tblClusters->setHorizontalHeaderLabels(labels);

    QString object_id = ui->txtObjectId->text();
    QString object_class = ui->cmbClass->currentText();
    ui->tblClusters->insertRow ( ui->tblClusters->rowCount() );
    ui->tblClusters->setItem   ( ui->tblClusters->rowCount()-1,
                                 0,
                                 new QTableWidgetItem(object_class));
    ui->tblClusters->setItem   ( ui->tblClusters->rowCount()-1,
                                 1,
                                 new QTableWidgetItem(object_id));
    ui->tblClusters->setItem   ( ui->tblClusters->rowCount()-1,
                                 2,
                                 new QTableWidgetItem("0"));
    ui->tblClusters->selectRow(ui->tblClusters->rowCount() - 1);
    saveCurrentCluster();
    colorizeCloud(*cloud,255,255,255,255);
    currentCluster.objectid = object_id.toStdString();
    currentCluster.oclass = object_class.toStdString();
    on_txtObjectId_textChanged(QString("boo"));
}


void CloudViewer::fileItemChanged()
{
    loadFile ( ui->listWidget_files->currentItem()->text().toStdString() );
}

void CloudViewer::painted ( double x, double y, double z , const long pointid, bool painting)
{
    movePointToAnn(x, y, z, pointid, painting);

}



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudViewer::colorize(pcl::PointCloud<pcl::PointXYZ> &source, int r, int g, int b)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_p;
    new_p.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    for (auto &pt: source.points) {
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

void CloudViewer::movePointToAnn ( double x, double y, double z , long pointid, bool painting)
{
    const int K=50;
    const float radius=ui->spPainter->value();
    std::vector<int> indices ( K );
    std::vector<float> distances ( K );
    pcl::PointXYZRGBA xyz;
    xyz.x = x; xyz.y =  y; xyz.z = z;
    bool allowStealing = ui->chkAllowStealing->isChecked();
    bool isClipping = ui->chkEnableClipping->isChecked();

    if (isClipping) {
        std::vector<pcl::visualization::Camera> cameras;
        viewer->getCameras(cameras);
        double cam_x = cameras[0].pos[0];
        double cam_y = cameras[0].pos[1];
        double cam_z = cameras[0].pos[2];
        double distance = sqrt(pow(cam_x - x,2) + pow(cam_y - y, 2) + pow(cam_z - z, 2));
        if (distance > ui->spMaxD->value() || distance < ui->spMinD->value()) {
            return;
        }
        if (x > ui->spMaxX->value() || x < ui->spMinX->value()) {
            return;
        }
        if (y > ui->spMaxY->value() || y < ui->spMinY->value()) {
            return;
        }
        if (z > ui->spMaxZ->value() || z < ui->spMinZ->value()) {
            return;
        }
    }

    // no updatePointCloud, too slow
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    auto cam = viewer->getCloudActorMap();
    pcl::visualization::CloudActorMap::iterator am_it =cam->find ("cloud");
    if (am_it == cam->end ()) {
        std::cout << "cloud not found???" << std::endl;
        return;
    }
    vtkSmartPointer<vtkPolyData> polydata = reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->GetInput ();
    vtkDataArray *scalars = polydata->GetPointData()->GetScalars();
    unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->GetPointer (0);


    auto nbResults = kdtree.radiusSearch ( xyz, radius, indices, distances, K );
    std::cout << "got " << nbResults << " points " << std::endl;
    if (nbResults  > 0 ) {
        for (int i=0; i<nbResults; i++ ) {
            auto &idx = indices.at(i);
            auto &pt = cloud->points.at(idx);
            //std::cout << "found point : " << (int)(pt.r) << " " << (int)(pt.g) << " " << (int)(pt.b) << " " << indices.at(0) << " " << pointid <<  std::endl;
            if (pt.r == pasCluster_R && pt.g == pasCluster_G && pt.b == pasCluster_B && (!allowStealing)) {
                continue;
            }
            if (painting) {
                pt.r = curCluster_R;
                pt.g = curCluster_G;
                pt.b = curCluster_B;
            } else {
                pt.r = 255;
                pt.g = 255;
                pt.b = 255;
            }
            pt.a = 255;
            // update vtk data directly to avoid updatePointCloud which is slow
            size_t j = idx*4;
            colors[j] = pt.r;
            colors[j+1] = pt.g;
            colors[j+2] = pt.b;
            colors[j+3] = pt.a;

        }
        ui->btnSaveCurrent->setEnabled(true);
    }


    scalars->Modified();
    //viewer->updatePointCloud(cloud, "cloud");
    _renderWindow->Render();
    //ui->qvtkWidget->repaint();
    

}

void CloudViewer::saveCurrentCluster()
{
    if (currentCluster.objectid == "") {
        return;
    }
    std::vector<int> cluster_indices;
    for (size_t idx = 0; idx < cloud->points.size(); idx++) {
        auto &pt = cloud->points.at(idx);
        if (pt.r == curCluster_R && pt.g == curCluster_G && pt.b == curCluster_B) {
            cluster_indices.push_back(idx);
        }
    }
    segments[currentCluster] = cluster_indices;
    std::cout << "saved segment " << currentCluster.objectid << " with " << cluster_indices.size() << " points" << std::endl;
    for (int idx=0; idx < ui->tblClusters->rowCount() ; idx++) {
        auto oclass =  ui->tblClusters->item( idx,
                                              0)->text().toStdString();
        auto objectid =  ui->tblClusters->item( idx,
                                                1)->text().toStdString();
        if (oclass == currentCluster.oclass && objectid == currentCluster.objectid) {
            ui->tblClusters->item(idx, 2)->setText(QString::number(cluster_indices.size()));
        }
    }
    saveJson();
    ui->btnSaveCurrent->setEnabled(false);
}

void CloudViewer::loadCluster(std::string oclass, std::string objectid) {
    currentCluster.objectid = objectid;
    currentCluster.oclass = oclass;
    colorizeCloud(*cloud, 255,255,255,255);

    if (currentCluster.objectid == "") {
        return;
    }
    auto &idxs = segments[currentCluster];
    for (auto x: idxs) {
        auto &pt = cloud->points.at(x);
        pt.r = curCluster_R;
        pt.g = curCluster_G;
        pt.b = curCluster_B;
    }
    viewer->updatePointCloud(cloud,"cloud");
    _renderWindow->Render();
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
    ui->tblClusters->model()->removeRows(0, ui->tblClusters->rowCount());

    QStringList labels;
    labels << tr("Class") << tr("Id") << tr("#Points");
    ui->tblClusters->setHorizontalHeaderLabels(labels);

    std::string fn = file_name;
    fn += "_labels.json";
    std::ifstream f(fn.c_str());
    if (f) {
        json j;
        f >> j;
        segments = j.get<decltype(segments)>();
        for (auto s: segments) {

            ui->tblClusters->insertRow ( ui->tblClusters->rowCount() );
            ui->tblClusters->setItem   ( ui->tblClusters->rowCount()-1,
                                         0,
                                         new QTableWidgetItem(QString::fromStdString(s.first.oclass)));
            ui->tblClusters->setItem   ( ui->tblClusters->rowCount()-1,
                                         1,
                                         new QTableWidgetItem(QString::fromStdString(s.first.objectid)));
            ui->tblClusters->setItem   ( ui->tblClusters->rowCount()-1,
                                         2,
                                         new QTableWidgetItem(QString::number(s.second.size())));
        }
    }


    colorizeCloud(*cloud, 255,255,255,255);

    viewer->addPointCloud ( cloud, rgba, "cloud" );

    //viewer->addCoordinateSystem();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid ( *cloud, centroid );

    //interactorStyle->GetInteractor()->FlyTo(interactorStyle->GetCurrentRenderer(), centroid[0], centroid[1], centroid[2]);
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

    std::cout << "centroid: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
    _renderWindow->Render();
    ui->qvtkWidget->update();
    ui->chkEnableClipping->setChecked(false);
    resetClippingSpinboxes();

}



void CloudViewer::on_spPointPicker_valueChanged(double arg1)
{
    interactorStyle->setPointPickerTolerance(arg1);
    interactorStyle->setPainterTolerance(arg1);
}

void CloudViewer::on_tblClusters_itemSelectionChanged()
{
    auto r = ui->tblClusters->selectionModel()->selectedRows();
    if (r.size() > 0) {
        int row = r.at(0).row();
        auto oclass =  ui->tblClusters->item( row,
                                              0)->text().toStdString();
        auto objectid =  ui->tblClusters->item( row,
                                                1)->text().toStdString();
        saveCurrentCluster();
        loadCluster(oclass, objectid);
    }
}

void CloudViewer::on_txtObjectId_textChanged(const QString &arg1)
{
    std::string oid = ui->txtObjectId->text().toStdString();
    std::string oclass = ui->cmbClass->currentText().toStdString();
    bool okay = true;
    if (oid == "" || oclass == "") {
        okay = false;
    }
    if (hasSegment(oclass,oid)) {
        okay = false;
    }
    ui->btnStartStop->setEnabled(okay);
}

void CloudViewer::on_cmbClass_currentTextChanged(const QString &arg1)
{
    on_txtObjectId_textChanged(arg1);
}

void CloudViewer::on_btnSaveCurrent_pressed()
{
    saveCurrentCluster();
}
