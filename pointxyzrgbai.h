#define PCL_NO_PRECOMPILE

#ifndef POINTXYZRGBAI_H
#define POINTXYZRGBAI_H
#endif

#include <pcl/point_types.h>

struct PointXYZRGBAI{
  PCL_ADD_POINT4D;
  union{
    struct{
      float intensity;
      uint32_t rgba;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; //EIGEN_ALIGN_16;

inline std::ostream& operator << (std::ostream& os, const PointXYZRGBAI& p);
