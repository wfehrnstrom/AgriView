#include "LidarFile.h"

const std::string LidarFile::valid_exts[] = {".pcd", ".las", ".laz", ".ascii"};
const std::string LidarFile::valid_operations[] = {"filters.decimation"};
const std::array<std::string, 1> LidarFile::valid_operation_extensions = {"_decimated"};
const std::string LidarFile::pcdExt = ".pcd";
const boost::filesystem::path LidarFile::inputDirectory = static_cast<std::string>("..") + boost::filesystem::path::preferred_separator + static_cast<std::string>("in") + boost::filesystem::path::preferred_separator;
const boost::filesystem::path LidarFile::outputDirectory = static_cast<std::string>("..") + boost::filesystem::path::preferred_separator + static_cast<std::string>("out") + boost::filesystem::path::preferred_separator;

PointXYZRGBAI EIGEN_ALIGN_16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBAI,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint32_t, rgba, rgba)
)

inline std::ostream& operator << (std::ostream& os, const PointXYZRGBAI& p){
  os << "(" << p.x << ", " << p.y << ", " << p.z << " - " << p.intensity << " - " << p.rgba << ")";
  return (os);
}

namespace std{
  template<> struct std::less<LidarFile>{
    bool operator() (const LidarFile& lhs, const LidarFile& rhs) const {
      return (lhs.getFileName().compare(rhs.getFileName()));
    }
  };
}

LidarFile::LidarFile(std::string filename, std::string parent_directory){
  ext = filename.substr(filename.find("."), std::string::npos);
  derivatives = new std::vector<LidarFile>();
  tiles = new std::vector<LidarFile>();
  fileName = filename;
  std::cout << "File Name: " << fileName << std::endl;
  if(!parent_directory.empty()){
    parentDirectoryPath = parent_directory;
  }
  offspring = checkForParent();
  filePath = findInputFile();
  std::cout << "File Path: " << filePath << std::endl;
  ext = filename.substr(fileName.find("."), std::string::npos);
  std::cout << "Extension: " << ext << std::endl;
  pcdFileName = getPCDFileName();
  std::cout << "PCD File Name: " << getPCDFileName() << std::endl;
  bool isValid = validate();
  initPCDFilePath();
  makeDirectory();
  //Do not set PCD File Path if it already has been initialized to some value
  std::cout << "PATH: " << getPCDFilePath() << std::endl;
  if(shouldTranslate()){
    std::cout << "should translate." << std::endl;
    translate(getPCDFilePath());
  }
  boost::filesystem::path pcdFileDirectory(getPCDFilePath().string().substr(0, getPCDFilePath().string().find_last_of(boost::filesystem::path::preferred_separator)));
  // if(!hasParent()){
  //   decimate();
  // }
  if(!hasParent()){
    checkForPre_ExistingDerivatives();
  }
  if(shouldTile()){
    std::cout << "Tiling." << std::endl;
    tiles = (tile(200000));
    derivatives = tiles;
  }
}

boost::filesystem::path LidarFile::parseForFileName(boost::filesystem::path path){
  boost::filesystem::path name(path.string().substr(path.string().rfind(boost::filesystem::path::preferred_separator) + 1));
  return name;
}

void LidarFile::checkForPre_ExistingDerivatives(){
  boost::filesystem::path pcdFileDirectory(getPCDFilePath().string().substr(0, getPCDFilePath().string().find_last_of(boost::filesystem::path::preferred_separator)));
  for(boost::filesystem::directory_iterator itr(pcdFileDirectory); itr != boost::filesystem::directory_iterator(); ++itr){
    boost::filesystem::path currentObjPath = itr->path();
    boost::filesystem::path filename = parseForFileName(currentObjPath);
    boost::filesystem::path filenameWithoutExt = filename.string().substr(0, filename.string().rfind("_")); //Ext in this case being _1.pcd, _2.pcd .....
    if(filenameWithoutExt.string() == getPCDFileDirectory().string()){
        if(filename.string().find(pcdExt) == -1){
          filename.append(pcdExt);
        }
        LidarFile lidar(filename.string(), getPCDFileDirectory().string());
        derivatives->push_back(lidar);
        tiles->push_back(lidar);
      }
    // else{
    //   for(int i = 0; i < valid_operation_extensions.size(); i++){
    //     if(currentObjPath.string().find(valid_operation_extensions.at(i)) != -1){
    //       std::string name = currentObjPath.string().substr(currentObjPath.string().rfind(boost::filesystem::path::preferred_separator) + 1);
    //       if(name.find(pcdExt) == -1){
    //         name.append(pcdExt);
    //       }
    //       //TODO
    //       LidarFile lidar(name, getPCDFileDirectory().string());
    //       derivatives->push_back(lidar);
    //     }
    //   }
    // }
  }
  for(int i = 0; i < derivatives->size(); i++){
    std::cout << "Pre-existing derivative with name: " << derivatives->at(i).getPCDFileName() << std::endl;
  }
}

pcl::PointCloud<PointXYZRGBAI> LidarFile::toPointCloud(){
  pcl::PointCloud<PointXYZRGBAI> cloudRGBAI;
  //std::cout << "LOADING OF POINT CLOUD FROM PATH: " << pcdFilePath.string() << std::endl;
  pcl::io::loadPCDFile<PointXYZRGBAI>(pcdFilePath.string(), cloudRGBAI);
  for(int i = 0; i < cloudRGBAI.points.size(); i++){
    cloudRGBAI.points[i].x += shifted[0];
    cloudRGBAI.points[i].y += shifted[1];
    cloudRGBAI.points[i].z += shifted[2];
  }
  return cloudRGBAI;
}

std::vector<std::string> parseLineForFields(std::string line){
  std::vector<std::string> fields;
  int spacePos = line.find(" ", 0);
  while(line.find(" ", spacePos) != -1 && (line.find(" ") != line.length() - 1)){
    fields.push_back(line.substr(spacePos + 1, line.find(" ", spacePos + 1) - (spacePos + 1)));
    spacePos = line.find(" ", spacePos + 1);
  }
  return fields;
}

std::vector<std::string> LidarFile::getFields(){
  std::ifstream ifs(pcdFilePath.string());
  std::vector<std::string> fields;
  std::string line;
  while(ifs.good()){
    std::getline(ifs, line);
    if(line.find("FIELDS") != -1){
      //std::cout << "Line with FIELDS: " << line << std::endl;
      fields = parseLineForFields(line);
      break;
    }
  }
  return fields;
}

double LidarFile::getYMax(){
  pcl::PointCloud<PointXYZRGBAI> cloud = toPointCloud();
  double max = cloud.points[0].y;
  for(int i = 0; i < cloud.points.size(); i++){
    if(cloud.points[i].y > max){
      max = cloud.points[i].y;
    }
  }
  return max;
}

double LidarFile::getYMin(){
  pcl::PointCloud<PointXYZRGBAI> cloud = toPointCloud();
  double min = cloud.points[0].y;
  for(int i = 0; i < cloud.points.size(); i++){
    if(cloud.points[i].y < min){
      min = cloud.points[i].y;
    }
  }
  return min;
}

bool LidarFile::hasParent(){
  return offspring;
}

bool LidarFile::checkForParent(){
  if(fileName.find("_") != -1 && !parentDirectoryPath.string().empty()){ //if fileName contains _
    const std::string numberPortion = fileName.substr(fileName.rfind("_") + 1, fileName.rfind(".") - (fileName.rfind("_") + 1));
    std::string::const_iterator itr = numberPortion.begin();
    while(itr != numberPortion.end() && std::isdigit(*itr)){
      ++itr;
    }
    bool isDigit = !numberPortion.empty() &&  itr == numberPortion.end();
    if(isDigit){
      return true;
    }
    else{
      for(int i = 0; i < valid_operation_extensions.size(); i++){
        if(fileName.find(valid_operation_extensions.at(i)) != -1){
          return true;
        }
      }
    }
  }
  return false;
}

std::string LidarFile::validateFileName(std::string name){
  std::replace(name.begin(), name.end(), ' ', '-');
  return name;
}

bool LidarFile::validate(){ //Check if file extension is valid
  bool valid_extension = false;
  for(int i = 0; i < NUMBER_OF_COMPATIBLE_FILE_TYPES; i++){
    if(ext == valid_exts[i]){
      valid_extension = true;
      break;
    }
  }
  return true;
}

bool LidarFile::isValid(){
  return valid;
}

std::string LidarFile::translate(boost::filesystem::path pcdPath){
  boost::filesystem::path pcdfilepath;
  if(pcdPath.string().empty()){
    pcdfilepath = outputDirectory;
    pcdfilepath += getPCDFileName().substr(0, getPCDFileName().find("."));
    pcdfilepath += boost::filesystem::path::preferred_separator;
    pcdfilepath += getPCDFileName();
  }
  else{
    pcdfilepath = pcdPath;
  }
  if(ext != pcdExt && !hasParent()){
    if(!boost::filesystem::exists(pcdfilepath)){
      try{
        std::string cmd = "pdal translate ";
        cmd.append(filePath.string());
        cmd.append(" ");
        //pcdfilepath = outputDirectory.append(static_cast<boost::filesystem::path>(getPCDFileName())).string();
        cmd.append(pcdfilepath.string());
        system(cmd.c_str());
        return pcdfilepath.string();
      }
      catch(std::exception& e){
        std::cout << "File Translation operation from " << ext << "to " << pcdExt << " failed!" << std::endl;
        return pcdfilepath.string();
      }
    }
    else{
      return pcdfilepath.string();
    }
  }
  else{
    return filePath.string();
  }
}

boost::filesystem::path LidarFile::findInputFile(){
  boost::filesystem::path fullInputFilePath = inputDirectory;
  fullInputFilePath += static_cast<boost::filesystem::path>(fileName);
  if(!hasParent()){
    if(boost::filesystem::exists(fullInputFilePath)){
      return fullInputFilePath;
    }
    else{
      std::cout << "Input File Not Found with filename specified in LidarFile Constructor" << std::endl;
      throw std::exception();
    }
  }
  else{ //if hasParent
    boost::filesystem::path inputFileLocation = getPCDFilePath();
    std::cout << "Input File location: " << inputFileLocation.string() << std::endl;
    return inputFileLocation;
  }
}

std::string LidarFile::getFileName() const{
  return fileName;
}

std::string LidarFile::getPCDFileName(){
  if(!pcdFileName.empty()){
    return pcdFileName;
  }else{
    pcdFileName = fileName.substr(0, fileName.find_last_of("."));
    pcdFileName.append(pcdExt);
    return pcdFileName;
  }
}

boost::filesystem::path LidarFile::getPCDFileDirectory(){
  //std::cout << "Inside getPCDFileDirectory getPCDFilePath() is" << getPCDFilePath().string() << std::endl;
  boost::filesystem::path pcdFileDirectory(getPCDFilePath().string().substr(0, getPCDFilePath().string().rfind(boost::filesystem::path::preferred_separator) + 1));
  return pcdFileDirectory;
}

boost::filesystem::path LidarFile::approximatePCDFilePath(){
  boost::filesystem::path pcdPath;
  try{
    if(hasParent()){
      pcdPath.append(parentDirectoryPath.string());
      //Check if a the directory exists.  If it does, then check that it contains the pcdFile
      if(directoryEntryExists() && directoryHasFile(getPCDFileName())){
        std::string thisDirectoryName = fileName.substr(0, fileName.rfind(ext));
        pcdPath.append(thisDirectoryName);
      }
      pcdPath.append(getPCDFileName());
    }
    else{
      pcdPath.append(outputDirectory.string());
      if(directoryEntryExists()){
        boost::filesystem::path fileDir(getPCDFileName().substr(0, getPCDFileName().rfind(pcdExt)));
        pcdPath.append(fileDir.string());
      }
      pcdPath.append(getPCDFileName());
    }
    pcdFilePath = pcdPath;
  }
  catch(std::exception& e){
    std::cerr << "In approximatePCDFilePath(): Unable to approximate PCD File: " << getPCDFileName() << std::endl;
  }
  return pcdPath;
}


boost::filesystem::path LidarFile::initPCDFilePath(){
  //Return the current file path
  bool invalidPCDPath = false;
  if(!pcdFilePath.string().empty()){ //pcdFilePath already initialized
    if(!boost::filesystem::exists(pcdFilePath)){
      invalidPCDPath = true;
    }
  }
  if(ext == pcdExt || invalidPCDPath){ //initialize the pcdFilePath or try to find the correct one, if invalid
    try{
      for(boost::filesystem::recursive_directory_iterator recur_itr(outputDirectory); recur_itr != boost::filesystem::recursive_directory_iterator(); recur_itr++){
        if(recur_itr->path().string().find(getPCDFileName()) != -1){
          pcdFilePath = recur_itr->path();
          return recur_itr->path();
        }
      }
    }
    catch(const std::exception& e){
      std::cerr << "In initPCDFilePath(): Failed to find PCD File with extension: " << ext << "and file name: " << getPCDFileName() << std::endl;
      return approximatePCDFilePath();
    }
  }
  return approximatePCDFilePath();
}

boost::filesystem::path LidarFile::getPCDFilePath(){
  if(!boost::filesystem::exists(pcdFilePath)){
    initPCDFilePath();
  }
  return pcdFilePath;
}

bool LidarFile::directoryEntryExists(){
  boost::filesystem::path folderName = fileName.substr(0, fileName.find_last_of("."));
  boost::filesystem::path directory;
  if(!hasParent()){
    boost::filesystem::path outputDir(outputDirectory);
    directory += outputDir;
    directory.append(folderName.string());
  }
  else{
    boost::filesystem::path parentDir(parentDirectoryPath);
    directory = parentDir.append(folderName.string());
  }
  return boost::filesystem::is_directory(directory);
}

bool LidarFile::directoryHasFile(boost::filesystem::path filename){
  //std::cout << "suh." << std::endl;
  boost::filesystem::path pcdFileDirectory(getPCDFilePath().string().substr(0, getPCDFilePath().string().find_last_of(boost::filesystem::path::preferred_separator)));
  //std::cout << "suh." << std::endl;
  // std::cout << "IN directoryHasFile.  pcdFileDirectory = " << pcdFileDirectory << std::endl;
  // std::cout << "IN directoryHasFile.  filePath = " << pcdFileDirectory.append(filename.string()) << std::endl;
  if(boost::filesystem::exists(pcdFileDirectory.append(filename.string()))){
    return true;
  }
  return false;
}

size_t LidarFile::numObjectsInDirectory(){
  boost::filesystem::path pcdDir = getPCDFilePath();
  boost::filesystem::path pcdFileDir(getPCDFilePath().string().substr(0, getPCDFilePath().string().find_last_of(boost::filesystem::path::preferred_separator)));
  size_t file_count = 0;
  for(boost::filesystem::directory_iterator itr(pcdFileDir); itr != boost::filesystem::directory_iterator(); ++itr){
    file_count++;
  }
  // for(auto &file : boost::filesystem::directory_iterator(pcdFileDir)){
  //   std::cout << file.path() << std::endl;
  //   file_count++;
  // }
  return file_count;

}

size_t LidarFile::numObjectsInDirectoryWithExtension(std::string extension){
  boost::filesystem::path pcdFileDirectory(getPCDFilePath().string().substr(0, getPCDFilePath().string().find_last_of(boost::filesystem::path::preferred_separator)));
  size_t file_count = 0;
  for(boost::filesystem::directory_iterator itr(pcdFileDirectory); itr != boost::filesystem::directory_iterator(); ++itr){
    if(itr->path().string().find(extension, itr->path().string().length() - extension.length()) != -1){
      file_count++;
    }
  }
  return file_count;
}

bool LidarFile::makeDirectory(){
  try{
    boost::filesystem::path folderName = fileName.substr(0, fileName.find_last_of("."));
    boost::filesystem::path parentDir(parentDirectoryPath);
    boost::filesystem::path outputDir(outputDirectory);
    if(hasParent()){ //place the directory to make in the parent directory
      boost::filesystem::path dirToMake = parentDir.append(folderName.string());
      if(!directoryEntryExists()){
        std::cout << "Directory does not already exist!" << std::endl;
        std::cout << dirToMake << std::endl;
        boost::filesystem::create_directory(dirToMake);
        movePCDFile(dirToMake);
      }
      else if(directoryEntryExists() && !directoryHasFile(getPCDFileName())){
        std::cout << "Directory exists but does not have file: " << getPCDFileName() << std::endl;
        std::cout << "ABCDEFGHIJK: " << dirToMake << std::endl;
        movePCDFile(dirToMake);
      }
    }
    else{
      if(!directoryEntryExists()){
        std::cout << "Wubalubadubdub" << std::endl;
        boost::filesystem::create_directory(outputDir.append(folderName.string()));
      }
      else{
        std::cout << "Directory already exists!" << std::endl;
        return false;
      }
    }
    return true;
  }catch(const std::exception& e){
    std::cout << "Directory not created." << std::endl;
    return false;
  }
}

bool LidarFile::movePCDFile(boost::filesystem::path newDirectory){
  try{
    boost::filesystem::path fullPath = newDirectory;
    std::cout << "OLD PATH: " << getPCDFilePath() << std::endl;
    std::cout << "NEW PATH: " << fullPath << std::endl;
    fullPath.append(getPCDFileName());
    boost::filesystem::rename(getPCDFilePath(), fullPath);
    pcdFilePath = fullPath;
    std::cout << "Movement of PCD file to path: " << fullPath << std::endl;
    return true;
  }
  catch(const std::exception& e){
    std::cout << "Movement of PCD file from path: " << pcdFilePath.string() << "to new path: " << newDirectory.append(pcdFileName) << " failed!" << std::endl;
    return false;
  }
}

uintmax_t LidarFile::size(){
  boost::system::error_code e;
  return boost::filesystem::file_size(filePath, e);
}

bool LidarFile::shouldTile(){
  if(size() > 5000000 && !hasParent()){
    if(numObjectsInDirectory() <= ((1 + derivatives->size()) - tiles->size())){
      return true;
    }
  }
  return false;
}

bool LidarFile::shouldTranslate(){
  if(directoryHasFile(pcdFileName)){
    return false;
  }
  else{
    return true;
  }
}

std::vector<LidarFile>* LidarFile::getTiles(){
  return tiles;
}

std::vector<LidarFile>* LidarFile::getDerivatives(){
  return derivatives;
}
//Iterate through tiles to find the index of the file with a name
int LidarFile::getIndexOfFileWithName(std::string name, std::vector<LidarFile>* vec){
  for(int i = 0; i < vec->size(); i++){
    if(vec->at(i).getPCDFileName() == name){
      return i;
    }
  }
  return -1;
}

std::vector<LidarFile>* LidarFile::getAllFilesMatchingSubstringArrayInDerivatives(std::array<std::string, 1> arr){
  std::vector<LidarFile>* vec = new std::vector<LidarFile>();
  for(int i = 0; i < derivatives->size(); i++){
    for(int j = 0; j < arr.size(); j++){
      if(derivatives->at(i).getPCDFileName().find(arr.at(j)) != -1){
        vec->push_back(derivatives->at(i));
      }
    }
  }
  return vec;
}

std::vector<LidarFile>* LidarFile::tile(size_t capacity){
  std::vector<LidarFile>* newFiles = new std::vector<LidarFile>();
  std::string newFileName;
  try{
     std::string cmd = "pdal split --capacity ";
     cmd.append(std::to_string(capacity));
     cmd.append(" ");
     for(int i = 0; i < 2; i++){
       cmd.append(pcdFilePath.string().c_str());
       cmd.append(" ");
     }
     system(cmd.c_str());
     boost::filesystem::path pcdFileDirectory(pcdFilePath.string().substr(0, pcdFilePath.string().find_last_of(boost::filesystem::path::preferred_separator)));
     int i = 0;
     for(boost::filesystem::directory_iterator itr(pcdFileDirectory); itr != boost::filesystem::directory_iterator(); ++itr){
       newFileName = itr->path().string().substr(itr->path().string().find_last_of(boost::filesystem::path::preferred_separator) + 1);
       std::vector<LidarFile>* operation_derivatives = getAllFilesMatchingSubstringArrayInDerivatives(valid_operation_extensions);
       bool foundInOperationDerivatives = false;
       for(int j = 0; j < operation_derivatives->size(); j++){
         if(newFileName == operation_derivatives->at(j).getPCDFileName()){
           foundInOperationDerivatives = true;
         }
       }
       if((itr->path().string().find(".pcd") != -1 && (itr->path().string() != pcdFilePath)) && !foundInOperationDerivatives){ //itr[i] is a pcd file
         boost::filesystem::path newFileNameCast = newFileName;
         LidarFile lidar(newFileNameCast.string(), pcdFileDirectory.string());
         newFiles->push_back(lidar);
         i++;
       }
     }
     for(int i = 1; i < newFiles->size(); i++){
       int lidarFileIndex = std::stoi(newFiles->at(i).getPCDFileName().substr(newFiles->at(i).getPCDFileName().rfind("_") + 1, newFiles->at(i).getPCDFileName().rfind(".") - (newFiles->at(i).getPCDFileName().rfind("_") + 1))) - 1;
       std::string lastSequentialFileName = newFiles->at(i).getPCDFileName().substr(0, newFiles->at(i).getPCDFileName().rfind("_") + 1).append(std::to_string(lidarFileIndex)).append(".pcd");
       newFiles->at(i).shift(0, newFiles->at(getIndexOfFileWithName(lastSequentialFileName, newFiles)).getYMax(), 0);
     }
     return newFiles;
  }
  catch(std::exception& e){
    std::cout << "Could not tile file " << fileName << std::endl;
    return newFiles;
  }
}

std::vector<LidarFile>* LidarFile::split(size_t capacity){
  std::vector<LidarFile>* newFiles = new std::vector<LidarFile>();
  std::string newFileName;
  try{
     std::string cmd = "pdal split --capacity ";
     cmd.append(std::to_string(capacity));
     cmd.append(" ");
     for(int i = 0; i < 2; i++){
       cmd.append(pcdFilePath.string().c_str());
       cmd.append(" ");
     }
     system(cmd.c_str());
     boost::filesystem::path pcdFileDirectory(pcdFilePath.string().substr(0, pcdFilePath.string().find_last_of(boost::filesystem::path::preferred_separator)));
     int i = 0;
     for(boost::filesystem::directory_iterator itr(pcdFileDirectory); itr != boost::filesystem::directory_iterator(); ++itr){
       newFileName = itr->path().string().substr(itr->path().string().find_last_of(boost::filesystem::path::preferred_separator) + 1);
       std::vector<LidarFile>* operation_derivatives = getAllFilesMatchingSubstringArrayInDerivatives(valid_operation_extensions);
       bool foundInOperationDerivatives = false;
       for(int j = 0; j < operation_derivatives->size(); j++){
         if(newFileName == operation_derivatives->at(j).getPCDFileName()){
           foundInOperationDerivatives = true;
         }
       }
       if((itr->path().string().find(".pcd") != -1 && (itr->path().string() != pcdFilePath)) && !foundInOperationDerivatives){ //itr[i] is a pcd file
         boost::filesystem::path newFileNameCast = newFileName;
         LidarFile lidar(newFileNameCast.string(), pcdFileDirectory.string());
         newFiles->push_back(lidar);
         i++;
       }
     }
     return newFiles;
  }
  catch(std::exception& e){
    std::cout << "Could not tile file " << fileName << std::endl;
    return newFiles;
  }
}

void LidarFile::shift(double x, double y, double z){
  shifted[0] = x; shifted[1] = y; shifted[2] = z;
}

void LidarFile::addShift(double x, double y, double z){
  shifted[0] += x; shifted[1] += y; shifted[2] += z;
}

double* LidarFile::getShifts(){
  return shifted;
}

void LidarFile::writeDecimationFilter(std::ofstream& ofs, std::string operation){
  std::string currentIndent = "";
  ofs << "{" << std::endl;
  currentIndent.append("\t");
  ofs << currentIndent << "\"pipeline\":[" << std::endl;
  currentIndent.append("\t");
  ofs << currentIndent << "\"" << getPCDFilePath().string() << "\",\n" << currentIndent << "{" << std::endl;
  currentIndent.append("\t");
  ofs << currentIndent << "\"type\":\"" << operation << "\"," << std::endl;
  ofs << currentIndent << "\"steps\":" << NUM_STEPS << std::endl;
  currentIndent.pop_back();
  ofs << currentIndent << "}," << std::endl;
  ofs << currentIndent << "\"" << getPCDFilePath().string().substr(0, getPCDFilePath().string().find_last_of(boost::filesystem::path::preferred_separator)+ 1).append(pcdFileName.substr(0, getPCDFileName().find(".")).append("_decimated").append(".pcd")) << "\"" << std::endl;
  currentIndent.pop_back();
  ofs << currentIndent << "]" << std::endl;
  ofs << "}" << std::endl;
}

bool LidarFile::createJSONFileForCurrentDir(std::string filename, std::string operation){
  bool canCreateJSONFile = false;
  for(int i = 0; i < NUMBER_OF_VALID_JSON_OPERATIONS; i++){
    if(operation == valid_operations[i]){
      canCreateJSONFile = true;
      break;
    }
  }
  if(canCreateJSONFile){
    std::ofstream ofs(filename);
    if(std::strcmp(operation.c_str(), "filters.decimation") == 0){
      writeDecimationFilter(ofs, operation);
      return true;
    }
  }
  else{
    std::cout << "Cannot create JSON operation file for operation type: " << operation << std::endl;
    throw std::exception();
  }
  return false;
}

void LidarFile::decimate(){
  std::string path = getPCDFilePath().string().substr(0, getPCDFilePath().string().find_last_of(boost::filesystem::path::preferred_separator) + 1).append("decimate.json");
  createJSONFileForCurrentDir(getPCDFilePath().string().substr(0, getPCDFilePath().string().find_last_of(boost::filesystem::path::preferred_separator) + 1).append("decimate.json"), "filters.decimation");
  std::string cmd = "pdal pipeline ";
  std::string derivedFileName = getPCDFileName().substr(0, getPCDFileName().rfind(pcdExt)).append("_decimated.pcd");
  std::cout << "DECIMATED FILE NAME: " << derivedFileName << std::endl;
  cmd.append(getPCDFilePath().string().substr(0, getPCDFilePath().string().find_last_of(boost::filesystem::path::preferred_separator) + 1).append("decimate.json"));
  try{
    system(cmd.c_str());
    LidarFile file(derivedFileName, getPCDFileDirectory().string());
    derivatives->push_back(file);
  }catch(std::exception e){
    std::cout << "Command: " << cmd << "Failed!" << std::endl;
  }
}
