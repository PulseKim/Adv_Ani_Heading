#ifndef _SKELGEN_H_
#define _SKELGEN_H_
#include <iostream>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/utils/utils.hpp"

using namespace dart::dynamics;

class SkelGen{
public:
  /// Constructor
  SkelGen();

  ~SkelGen();
  

  BodyNode* freeCylinder(const SkeletonPtr& skel, const std::string& name, double rad, double height, const Eigen::Vector3d offChild, double mass, const Eigen::Vector3d color);
  BodyNode* freeCylinder(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, double rad, double height,   const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* freeBox(const SkeletonPtr& skel, const std::string& name, const Eigen::Vector3d boxsize, const Eigen::Vector3d offChild, double mass, const Eigen::Vector3d color);
  BodyNode* freeBox(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d boxsize, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* freeSphere(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, double radius, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* freeSphere(const SkeletonPtr& skel, const std::string& name, double radius, const Eigen::Vector3d offChild, double mass, const Eigen::Vector3d color);

  BodyNode* weldBox(const SkeletonPtr& skel, const std::string& name, const Eigen::Vector3d boxsize, const Eigen::Vector3d offChild, double mass, const Eigen::Vector3d color);  
  BodyNode* weldBox(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d boxsize, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* weldSphere(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, double radius, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);


  BodyNode* ballBox(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, Eigen::Vector3d boxsize, Eigen::Vector3d offChild, Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* ballCylinder(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, double rad, double height, Eigen::Vector3d offChild, Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* ballEllipsoid(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, Eigen::Vector3d ellipsoidsize, Eigen::Vector3d offChild, Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);

  BodyNode* univBox(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis1, const Eigen::Vector3d axis2,Eigen::Vector3d boxsize, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* univCylinder(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis1, const Eigen::Vector3d axis2, double rad, double height, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* univEllipsoid(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis1, const Eigen::Vector3d axis2, Eigen::Vector3d ellipsoidsize, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass,const  Eigen::Vector3d color);

  BodyNode* revolBox(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis, const Eigen::Vector3d boxsize, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* revolCylinder(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis, double rad, double height, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* revolEllipsoid(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis, const Eigen::Vector3d ellipsoidsize, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);
  BodyNode* revolCapsule(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis, double rad, double height, const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color);


};
#endif