#include <iostream>
#include "SkelGen.h"

SkelGen::SkelGen(){}
SkelGen::~SkelGen(){}

//WeldJoint Overriding with different parameters
BodyNode*
SkelGen::weldBox
(const SkeletonPtr& skel, const std::string& name, 
	const Eigen::Vector3d boxsize, const Eigen::Vector3d offChild, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(boxsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	WeldJoint::Properties props;
	props.mName = name;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	bn = skel->createJointAndBodyNodePair<WeldJoint>(nullptr, props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::weldBox
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d boxsize, 
	const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(boxsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	WeldJoint::Properties props;
	props.mName = name;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<WeldJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::weldSphere
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, double radius, 
	const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<SphereShape>(new SphereShape(radius));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	WeldJoint::Properties props;
	props.mName = name;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<WeldJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode*
SkelGen::freeBox
(const SkeletonPtr& skel, const std::string& name, const Eigen::Vector3d boxsize, const Eigen::Vector3d offChild, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(boxsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	FreeJoint::Properties props;
	props.mName = name;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	bn = skel->createJointAndBodyNodePair<FreeJoint>(nullptr, props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::freeBox
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d boxsize, 
	const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(boxsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	FreeJoint::Properties props;
	props.mName = name;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<FreeJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::freeCylinder
(const SkeletonPtr& skel, const std::string& name, double rad, double height, 
	const Eigen::Vector3d offChild, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<CylinderShape>(new CylinderShape(rad, height));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	FreeJoint::Properties props;
	props.mName = name;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	bn = skel->createJointAndBodyNodePair<FreeJoint>(nullptr,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}


BodyNode* 
SkelGen::freeCylinder
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, double rad, double height, 
	const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<CylinderShape>(new CylinderShape(rad, height));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	FreeJoint::Properties props;
	props.mName = name;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<FreeJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::freeSphere
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, double radius, 
	const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<SphereShape>(new SphereShape(radius));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	FreeJoint::Properties props;
	props.mName = name;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<FreeJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::freeSphere
(const SkeletonPtr& skel, const std::string& name, double radius, 
	const Eigen::Vector3d offChild, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<SphereShape>(new SphereShape(radius));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	FreeJoint::Properties props;
	props.mName = name;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	bn = skel->createJointAndBodyNodePair<FreeJoint>(nullptr,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}





//BallJoint Overriding with different parameters

BodyNode* 
SkelGen::ballBox
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d boxsize, 
	const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(boxsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	BallJoint::Properties props;
	props.mName = name;
	props.mDampingCoefficients = Eigen::Vector3d::Constant(0.2);

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<BallJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode*
SkelGen::ballCylinder
(const SkeletonPtr &skel, BodyNode *parent, const std::string &name, double rad, double height, 
	const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<CylinderShape>(new CylinderShape(rad,height));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	BallJoint::Properties props;
	props.mName = name;
	props.mDampingCoefficients = Eigen::Vector3d::Constant(0.2);

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<BallJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::ballEllipsoid
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d ellipsoidsize, 
	const Eigen::Vector3d offChild, const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<EllipsoidShape>(new EllipsoidShape(ellipsoidsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	BallJoint::Properties props;
	props.mName = name;
	props.mDampingCoefficients = Eigen::Vector3d::Constant(0.2);


	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<BallJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}



BodyNode* 
SkelGen::univBox
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis1, 
	const Eigen::Vector3d axis2, const Eigen::Vector3d boxsize, const Eigen::Vector3d offChild,
	const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(boxsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	UniversalJoint::Properties props;
	props.mName = name;
	props.mAxis[0] = axis1;
	props.mAxis[1] = axis2;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<UniversalJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode*
SkelGen::univCylinder
(const SkeletonPtr &skel, BodyNode *parent, const std::string &name, const Eigen::Vector3d axis1, 
	const Eigen::Vector3d axis2, double rad, double height, const Eigen::Vector3d offChild, 
	const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<CylinderShape>(new CylinderShape(rad,height));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	UniversalJoint::Properties props;
	props.mName = name;
	props.mAxis[0] = axis1;
	props.mAxis[1] = axis2;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<UniversalJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::univEllipsoid
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis1, 
	const Eigen::Vector3d axis2, const Eigen::Vector3d ellipsoidsize, const Eigen::Vector3d offChild,
	const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<EllipsoidShape>(new EllipsoidShape(ellipsoidsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	UniversalJoint::Properties props;
	props.mName = name;
	props.mAxis[0] = axis1;
	props.mAxis[1] = axis2;


	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<UniversalJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::revolBox
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis,
	const Eigen::Vector3d boxsize, const Eigen::Vector3d offChild,
	const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(boxsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	RevoluteJoint::Properties props;
	props.mName = name;
	props.mAxis = axis;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode*
SkelGen::revolCylinder
(const SkeletonPtr &skel, BodyNode *parent, const std::string &name, const Eigen::Vector3d axis, 
	double rad, double height, const Eigen::Vector3d offChild, 
	const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<CylinderShape>(new CylinderShape(rad,height));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	RevoluteJoint::Properties props;
	props.mName = name;
	props.mAxis = axis;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode* 
SkelGen::revolEllipsoid
(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, const Eigen::Vector3d axis, 
	const Eigen::Vector3d ellipsoidsize, const Eigen::Vector3d offChild,
	const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<EllipsoidShape>(new EllipsoidShape(ellipsoidsize));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	RevoluteJoint::Properties props;
	props.mName = name;
	props.mAxis = axis;


	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}

BodyNode*
SkelGen::revolCapsule
(const SkeletonPtr &skel, BodyNode *parent, const std::string &name, const Eigen::Vector3d axis, 
	double rad, double height, const Eigen::Vector3d offChild, 
	const Eigen::Vector3d offParent, double mass, const Eigen::Vector3d color)
{
	//Shape
	ShapePtr shape = std::shared_ptr<CapsuleShape>(new CapsuleShape(rad,height));

	//Inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	//Joint Parsing
	BodyNode* bn;
	RevoluteJoint::Properties props;
	props.mName = name;
	props.mAxis = axis;

	Eigen::Isometry3d T1;
	T1.setIdentity();	
	T1.translation() = offChild;
	props.mT_ChildBodyToJoint = T1;

	Eigen::Isometry3d T2;
	T2.setIdentity();
	T2.translation() = offParent;
	props.mT_ParentBodyToJoint = T2;

	bn = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,props,BodyNode::AspectProperties(name)).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	auto visualShapenodes = bn->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(color);
	bn->setInertia(inertia);	
	return bn;
}