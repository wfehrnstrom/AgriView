#include "Map.h"
// template<> std::less<pcl::PointCloud<pcl::PointXYZRGB> > {
//   operator() (const pcl::PointCloud<pcl::PointXYZRGB>& pc1, const pcl::PointCloud<pcl::PointXYZRGB>& pc2) const {
//     return (reinterpret_cast<size_t>(pc1) < reinterpret_cast<size_t>(pc2));
//   }
// };


POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBAI,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint32_t, rgba, rgba)
)

Map::Map(std::string fileName) : pclView(new pcl::visualization::PCLVisualizer("Point Cloud View")){
  id = reinterpret_cast<size_t>(this);
  ColorMode colorMode = real;
  LidarFile file(fileName);
  addFile(file, true); //true refers to turning visualization on
  initialize();
  setvIndexNDVI();
  ColorMode mode = ndvi;
  setColorMode(mode);
  std::cout << "about to view." << std::endl;
  view();
}

Map::Map(LidarFile file){
  //TODO
}

Map::Map(std::vector<LidarFile> files){
  //TODO
}

size_t Map::getID() const{
  return id;
}

void Map::addFile(LidarFile file, bool visualize){
  addedFiles.push_back(file);
  std::cout << "File added to vector." << std::endl;
  pcl::PointCloud<PointXYZRGBAI> fullCloud;
  //NEED TO OPTIMIZE OR REMOVE
  pcl::io::loadPCDFile<PointXYZRGBAI>(file.getPCDFilePath().string(), fullCloud);
  masterClouds.insert(std::pair<std::string, pcl::PointCloud<PointXYZRGBAI> >(file.getPCDFileName(), fullCloud));
  if(visualize){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(file.getPCDFilePath().string(), cloud);
    associatedClouds.insert(std::pair<std::string, pcl::PointCloud<pcl::PointXYZRGB> >(file.getPCDFileName(), cloud));
    visualizedFiles.push_back(cloud);
  }
}

void keyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void* viewer_void){
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
  std::cout << "KEY PRESSED" << std::endl;
  if(event.getKeySym() == "w" && event.keyDown()){
    std::cout << "W PRESSED" << std::endl;
  }
}

void Map::initialize(){
  pclView->setBackgroundColor(0, 0, 0);
  pclView->addCoordinateSystem(1.0);
  //std::cout << associatedClouds.size() << std::endl;
  for(int i = 0; i < addedFiles.size(); i++){
    std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB> >::const_iterator itr = associatedClouds.find(addedFiles.at(i).getPCDFileName());
    if(itr != associatedClouds.end()){
      pcl::PointCloud<pcl::PointXYZRGB> currentCloud = associatedClouds.find(addedFiles.at(i).getPCDFileName())->second;
      std::cout << "currentFile initialized." << std::endl;
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudAtIndexConstCast(&currentCloud);
      std::cout << "Const Ptr cloud initialized." << std::endl;
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>* rgbHandler = new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloudAtIndexConstCast);
      pclView->addPointCloud<pcl::PointXYZRGB>(cloudAtIndexConstCast, *rgbHandler, addedFiles.at(i).getPCDFileName());
      std::cout << "Point Cloud added." << std::endl;
      pclView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, addedFiles.at(i).getPCDFileName());
      std::cout << "Point Cloud rendering properties set." << std::endl;
    }
  }
  pclView->registerKeyboardCallback(keyboardEventCallback, (void*)&pclView);
  pclView->initCameraParameters();
}

void Map::view(){
  while(!pclView->wasStopped()){
    pclView->spinOnce();
  }
}

void Map::setViewingPosition(double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z){
  std::vector<pcl::visualization::Camera> cameras;
  pclView->getCameras(cameras);
  cameras[0].pos[0] = pos_x;
  cameras[0].pos[1] = pos_y;
  cameras[0].pos[2] = pos_z;
  cameras[0].view[0] = view_x;
  cameras[0].view[1] = view_y;
  cameras[0].view[2] = view_z;
}

void Map::moveCameraBy(double amount_x, double amount_y, double amount_z){
  std::vector<pcl::visualization::Camera> cameras;
  pclView->getCameras(cameras);
  cameras[0].pos[0] += amount_x;
  cameras[0].pos[1] += amount_y;
  cameras[0].pos[2] += amount_z;
  pclView->setCameraPosition(cameras[0].pos[0], cameras[0].pos[1], cameras[0].pos[2], cameras[0].view[0], cameras[0].view[1], cameras[0].view[2]);
}

void Map::setvIndexNDVI(){
  vIndexes.resize(addedFiles.size());
  for(int i = 0; i < addedFiles.size(); i++){
    std::string key_accessor = addedFiles[i].getPCDFileName();
    std::cout << "Size: " << masterClouds[key_accessor].size() << std::endl;
    vIndexes[i].resize(masterClouds[key_accessor].size());
    for(int j = 0; j < masterClouds[key_accessor].size(); j++){
      uint32_t red = (masterClouds[key_accessor].points[j].rgba >> 16) & (0x0000ff);
      vIndexes[i][j].push_back(calculateNDVI(masterClouds[key_accessor].points[j].intensity, (float)red));
    }
  }
}

float Map::calculateNDVI(float nir, float r){
  std::cout << "NDVI: " << (nir - r)/(nir + r) << std::endl;
  return (nir - r)/(nir + r);
}

bool Map::setColorMode(ColorMode color){
  if(color == ndvi && currentColor != ndvi){
    return setColorModeNDVI();
  }
  else if(color == height && currentColor != height){
    return setColorModeHeight();
  }
  else if(color == real && currentColor != real){
    return true;
  }
  else{
    std::cout << "Invalid Color Type. Color not set." << std::endl;
  }
  return false;
}
// void Map::mouseEventCallback(const pcl::visualization::MouseEvent &event, void* viewer_void){
//
// }

bool Map::setColorModeNDVI(){
  try{
    std::cout << "Setting color mode to ndvi" << std::endl;
    for(int i = 0; i < addedFiles.size(); i++){
      if(associatedClouds.find(addedFiles[i].getPCDFileName()) != associatedClouds.end()){
          pcl::PointCloud<pcl::PointXYZRGB> cloud(associatedClouds.find(addedFiles[i].getPCDFileName())->second);
          pcl::PointCloud<PointXYZRGBAI> cloudInfo(masterClouds.find(addedFiles[i].getPCDFileName())->second);
          for(int j = 0; j < cloud.points.size(); j++){
            float intensity = cloudInfo.points[j].intensity;
            cloud.points[j].b = cloud.points[j].g;
            cloud.points[j].g = cloud.points[j].r;
            cloud.points[j].r = (uint32_t)intensity;
            //std::cout << "Point " << j + 1 << " Intensity: " << cloud.points[j].r << std::endl;
          }
          std::cout << "Finished changing color values." << std::endl;
          pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr constCloud(&cloud);
          std::cout << "Const Cloud created." << std::endl;
          pclView->removePointCloud(addedFiles[i].getPCDFileName());
          std::cout << "removed Point Cloud." << std::endl;
          pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>* rgbHandler = new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(constCloud);
          std::cout << "rgb handler created." << std::endl;
          pclView->addPointCloud(constCloud, *rgbHandler, addedFiles[i].getPCDFileName());
          std::cout << "added new Point Cloud." << std::endl;
          pclView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, addedFiles.at(i).getPCDFileName());
          std::cout << "Point Cloud rendering properties set." << std::endl;
      }
    }
    currentColor = ndvi;
    return true;
  }
  catch(std::exception& e){
    std::cout << "Color mode not set to NDVI." << std::endl;
  }
  return false;
}

bool Map::setColorModeReal(){
  try{
    for(int i = 0; i < addedFiles.size(); i++){
      if(associatedClouds.find(addedFiles[i].getPCDFileName()) != associatedClouds.end()){
        pcl::PointCloud<pcl::PointXYZRGB> cloud(associatedClouds.find(addedFiles[i].getPCDFileName())->second);
        pcl::PointCloud<PointXYZRGBAI> cloudInfo(masterClouds.find(addedFiles[i].getPCDFileName())->second);
        for(int j = 0; j < cloud.points.size(); j++){
          uint32_t red = (cloudInfo.points[j].rgba >> 16) & (0x0000ff);
          uint32_t green = (cloudInfo.points[j].rgba >> 8) & (0x0000ff);
          uint32_t blue = (cloudInfo.points[j].rgba) & (0x0000ff);
          cloud.points[j].r = red;
          cloud.points[j].g = green;
          cloud.points[j].b = blue;
        }
        std::cout << "Finished changing color values." << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr constCloud(&cloud);
        std::cout << "Const Cloud created." << std::endl;
        pclView->removePointCloud(addedFiles[i].getPCDFileName());
        std::cout << "removed Point Cloud." << std::endl;
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>* rgbHandler = new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(constCloud);
        std::cout << "rgb handler created." << std::endl;
        pclView->addPointCloud(constCloud, *rgbHandler, addedFiles[i].getPCDFileName());
        std::cout << "added new Point Cloud." << std::endl;
        pclView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, addedFiles.at(i).getPCDFileName());
        std::cout << "Point Cloud rendering properties set." << std::endl;
      }
    }
    currentColor = real;
    return true;
  }
  catch(std::exception& e){
    std::cout << "Color Mode not set to real." << std::endl;
  }
  return false;
}

bool Map::setColorModeHeight(){
  try{
    std::cout << "Setting color mode to height" << std::endl;
    for(int i = 0; i < addedFiles.size(); i++){
      if(associatedClouds.find(addedFiles[i].getPCDFileName()) != associatedClouds.end()){
        pcl::PointCloud<pcl::PointXYZRGB> cloud(associatedClouds.find(addedFiles[i].getPCDFileName())->second);
        pcl::PointCloud<PointXYZRGBAI> cloudInfo(masterClouds.find(addedFiles[i].getPCDFileName())->second);
        //Determine color by scaling z value to 32 bit number with formula xprime = a + ((x - xmin)*(b - a))/(xmax - xmin)
        pcl::PointXYZRGB maxPoint, minPoint;
        pcl::getMinMax3D(cloud, minPoint, maxPoint);
        for(int j = 0; j < cloud.size(); j++){
          double rgbpercentage = (cloud.points[j].z - minPoint.z)/(maxPoint.z - minPoint.z);
          //Red = rgbPercentage * 255;
          //Green = ((percentage of rgbPercentage in middle 50) - (percentage that isnt)) * 255;
          //Blue = (1 - rgbPercentage) * 255;
          //x - .75 + x - .5
          // uint32_t condensedRGBA = (uint32_t)rgba;
          cloud.points[j].r = (uint32_t)(rgbpercentage * 255);
          double outside, inBetween;
          if(rgbpercentage > 0.75){
            inBetween = fabs(rgbpercentage - 0.25) - (rgbpercentage - 0.75);
            outside = (rgbpercentage - 0.75) + 0.25;
          }
          else if(rgbpercentage > 0.5){
            inBetween = fabs(rgbpercentage - 0.25) - (rgbpercentage - 0.75);
            outside = 0.25;
          }
          else if(rgbpercentage < 0.5 && rgbpercentage > 0.25){
            inBetween = fabs(rgbpercentage - 0.25);
            outside = 0.25;
          }
          else{
            inBetween = 0;
            outside = rgbpercentage;
          }
          cloud.points[j].g = (uint32_t)((inBetween - outside) * 255);
          cloud.points[j].b = (uint32_t)((1 - rgbpercentage) * 255);
          // cloud.points[j].g = (condensedRGBA >> 8) & (0x0000ff);
          // cloud.points[j].b = (condensedRGBA) & (0x0000ff);

        }
        std::cout << "Finished changing color values." << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr constCloud(&cloud);
        std::cout << "Const Cloud created." << std::endl;
        pclView->removePointCloud(addedFiles[i].getPCDFileName());
        std::cout << "removed Point Cloud." << std::endl;
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>* rgbHandler = new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(constCloud);
        std::cout << "rgb handler created." << std::endl;
        pclView->addPointCloud(constCloud, *rgbHandler, addedFiles[i].getPCDFileName());
        std::cout << "added new Point Cloud." << std::endl;
        pclView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, addedFiles.at(i).getPCDFileName());
        std::cout << "Point Cloud rendering properties set." << std::endl;
      }
    }
    currentColor = height;
    return true;
  }
  catch(std::exception& e){
    std::cout << "Color Mode not set to height." << std::endl;
  }
  return false;
}
