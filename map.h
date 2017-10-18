#ifndef MAP_H
#define MAP_H
#endif

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/point_types.h>
#include <vector>
#include <map>
#include <utility>
#include "LidarFile.h"

template<> struct std::less<pcl::PointCloud<pcl::PointXYZRGB> > {
  bool operator() (const pcl::PointCloud<pcl::PointXYZRGB>& pc1, const pcl::PointCloud<pcl::PointXYZRGB>& pc2) const {
    return (reinterpret_cast<size_t>(&pc1) < reinterpret_cast<size_t>(&pc2));
  }
};

template<> struct std::less<pcl::PointCloud<PointXYZRGBAI> > {
  bool operator() (const pcl::PointCloud<PointXYZRGBAI>& pc1, const pcl::PointCloud<PointXYZRGBAI>& pc2) const {
    return (reinterpret_cast<size_t>(&pc1) < reinterpret_cast<size_t>(&pc2));
  }
};

class Map{

  public:
    Map(std::string fileName);
    Map(LidarFile file);
    Map(std::vector<LidarFile> files);
    enum ColorMode {real = 0, ndvi, height};
    ColorMode currentColor;
    bool setColorMode(ColorMode color);
    bool setColorModeNDVI();
    bool setColorModeHeight();
    bool setColorModeReal();
    void addFile(LidarFile file, bool visualize = true);
    void removeVisualizationOfFileName(std::string fileName);
    void removeVisualizationOfFile(LidarFile file);
    void removeLidarFile(LidarFile file);
    void removeLidarFile(std::string fileName);
    void setViewingPosition(double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z);
    void moveCameraBy(double amount_x, double amount_y, double amount_z);
    void initialize();
    void view();
    void pauseView();
    void stopView();
    size_t getID() const;

  private:
    uint32_t id;
    bool addCameraShortcut(const char c);
    bool addCameraShortcut(const char c, double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z);
    float calculateNDVI(float nir, float r);
    void setvIndexNDVI();
    std::vector<std::vector<std::vector<float> > > vIndexes; //0 is NDVI
    std::vector<std::vector<float*> > channels;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > visualizedFiles;
    std::vector<LidarFile> addedFiles;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB> > associatedClouds;
    //pcl::PointCloud<PointXYZRGBAI> cl;
    std::map<std::string, pcl::PointCloud<PointXYZRGBAI> > masterClouds;
    std::vector<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> > rgbHandlers;
    std::vector<char> reservedKeys;
    std::vector<char> validKeys;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pclView;
};
