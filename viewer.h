#ifndef VIEWER_H
#define VIEWER_H

// C++
#include <iostream>
// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QListWidgetItem>
#include <QProcess>
#include <QDirIterator>
// Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include "annotatorinteractor.h"
#include <tuple>
#include "json.hpp"
#include <qmenu.h>

using nlohmann::json;

typedef struct feature {
  int id;
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
} Feature;

namespace Ui {
  class CloudViewer;
}

#define TIED_OP(STRUCT, OP) \
    inline bool operator OP(const STRUCT& lhs, const STRUCT& rhs) \
    { \
        return lhs.tie() OP rhs.tie(); \
    }

#define TIED_COMPARISONS(STRUCT) \
    TIED_OP(STRUCT, ==) \
    TIED_OP(STRUCT, !=) \
    TIED_OP(STRUCT, <) \
    TIED_OP(STRUCT, <=) \
    TIED_OP(STRUCT, >=) \
    TIED_OP(STRUCT, >)

struct ClusterKey {
    std::string oclass;
    std::string objectid;
    std::tuple<std::string, std::string> tie() const { return std::tie(oclass, objectid); }

};

inline void to_json(json& j, const ClusterKey& p) {
    j = json{{"oclass", p.oclass}, {"objectid", p.objectid}};
}

inline void from_json(const json& j, ClusterKey& p) {
    j.at("oclass").get_to(p.oclass);
    j.at("objectid").get_to(p.objectid);
}


TIED_COMPARISONS(ClusterKey)


class CloudViewer: public QMainWindow {
  Q_OBJECT
  
 public:
  explicit CloudViewer(QWidget *parent = 0);
  bool hasSegment(std::string objectClass, std::string objectId);
  ~CloudViewer();
  
  public slots:
    void loadButtonClicked();
    void labelButtonClicked();
    void fileItemChanged();
    void painted(double x, double y, double z, long pointid, bool painting);
    
 protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorize(pcl::PointCloud<pcl::PointXYZ> &source, int r, int g, int b);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, cloud_ann;
    std::map<ClusterKey, std::vector<int>> segments; // todo must convert to std::set, will be much faster for point stealing ea
    void movePointToAnn(double x, double y, double z, long pointid, bool painting);
    void createSegmentFromAnn(std::string segment_name);
    void saveCurrentCluster();
    void loadCluster(std::string oclass, std::string objectid);
    void colorizeCloud(pcl::PointCloud<pcl::PointXYZRGBA> &c, char r, char g, char b, char a);
    void saveJson();
    void resetClippingSpinboxes();
    void updateCameraClipping();
    void stealPointsFromOtherClusters(ClusterKey k);

    void confirmDeleteCurrentCluster();
    void renameCurrentCluster();
    void flyToCluster(ClusterKey &key);
    void flyToSelectedCluster();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCluster(ClusterKey &key);

    void loadFile(std::string file_name);
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;


private slots:
    void on_btnSaveCurrent_pressed();
    void on_cmbClass_currentTextChanged(const QString &arg1);
    void on_txtObjectId_textChanged(const QString &arg1);
    void on_tblClusters_itemSelectionChanged();
    void on_spPointPicker_valueChanged(double arg1);

private:
    Ui::CloudViewer *ui;
    QString load_file_path, current_label_path;
    bool file_labeled;
    bool auto_next;
    
    bool removing_outliers;
    bool downsampling;
    bool removing_planes;
    bool is_dirty = false;
    
    vtkSmartPointer <vtkGenericOpenGLRenderWindow> _renderWindow;
    vtkSmartPointer<AnnotatorInteractor>  interactorStyle;

    std::vector<Feature> features;
    std::vector< std::vector<std::string> > labels;
    std::fstream label_file;
    ClusterKey currentCluster;

    const int curCluster_R = 255;
    const int curCluster_G = 0;

    const int curCluster_B = 0;

    const int pasCluster_R = 127;
    const int pasCluster_G = 127;
    const int pasCluster_B = 0;


    std::shared_ptr<QMenu> context;


};

#endif // VIEWER_H
