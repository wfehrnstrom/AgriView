#ifndef LIDARFILE_H
#define LIDARFILE_H
#endif

#include <boost/filesystem.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <iostream> //SCAFFOLDING ONLY
#include <vector>
#include <array>
#include <cstring>

#include "pointxyzrgbai.h"

#define NUMBER_OF_COMPATIBLE_FILE_TYPES 4
#define NUMBER_OF_VALID_JSON_OPERATIONS 1
#define NUM_STEPS 3

class LidarFile{

  public:
    LidarFile(std::string fileName, std::string parent_directory = ""); //STABLE
    uintmax_t size(); //STABLE //Returns file size, not point cloud size
    bool shouldTile(); //STABLE
    bool shouldTranslate();
    bool hasParent(); //STABLE //Returns true if the LidarFile is a derivative of a parent file.  If this is true, filePath and fileName will be linked to the pcdFile
    bool directoryHasFile(boost::filesystem::path filename); //STABLE
    boost::filesystem::path getPCDFileDirectory();
    boost::filesystem::path getPCDFilePath();
    boost::filesystem::path initPCDFilePath();
    boost::filesystem::path approximatePCDFilePath();
    void decimate(); //STABLE
    std::vector<LidarFile>* split(size_t capacity); //TODO
    std::vector<LidarFile>* tile(size_t capacity); //TODO
    std::vector<LidarFile>* getDerivatives(); //TODO
    std::vector<LidarFile>* getTiles(); //TODO
    double* getShifts(); //STABLE
    std::string getFileName() const; //STABLE
    std::string getPCDFileName(); //STABLE
    pcl::PointCloud<PointXYZRGBAI> toPointCloud(); //STABLE
    std::vector<std::string> getFields(); //STABLE
    double getYMax(); //STABLE
    double getYMin(); //STABLE
    bool isValid(); //STABLE
    bool validate(); //STABLE //validate sets the boolean valid after checking to make sure that the file is indeed valid by mapping the file extension to a valid extension
    void shift(double x, double y, double z); //STABLE
    void addShift(double x, double y, double z); //STABLE
  private:
    void checkForPre_ExistingDerivatives();
    boost::filesystem::path parseForFileName(boost::filesystem::path path);
    bool valid;
    static std::string validateFileName(std::string name);
    bool offspring;
    bool checkForParent();
    std::vector<LidarFile>* derivatives;
    std::vector<LidarFile>* tiles;
    std::string fileName;
    boost::filesystem::path filePath;
    boost::filesystem::path pcdFilePath;
    boost::filesystem::path parentDirectoryPath;
    std::string pcdFileName;
    std::string ext;
    static const std::string valid_exts[];
    static const std::string pcdExt;
    static const std::string valid_operations[];
    static const std::array<std::string, 1> valid_operation_extensions;
    static const boost::filesystem::path inputDirectory;
    static const boost::filesystem::path outputDirectory;
    void writeDecimationFilter(std::ofstream& ofs, std::string operation); //STABLE
    bool createJSONFileForCurrentDir(std::string name, std::string operation); //STABLE
    double shifted[3];
    bool makeDirectory(); //STABLE
    bool movePCDFile(boost::filesystem::path newDirectory); //TODO
    std::string translate(boost::filesystem::path pcdPath = ""); //TODO: Check actual translation path
    bool directoryEntryExists(); //STABLE
    boost::filesystem::path findInputFile(); //STABLE
    size_t numObjectsInDirectory(); //STABLE
    size_t numObjectsInDirectoryWithExtension(std::string extension); //STABLE
    int getIndexOfFileWithName(std::string name, std::vector<LidarFile>* vec);
    std::vector<LidarFile>* getAllFilesMatchingSubstringArrayInDerivatives(std::array<std::string, 1> array);
};
