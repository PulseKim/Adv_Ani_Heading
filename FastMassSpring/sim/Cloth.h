#ifndef __CLOTH_H__
#define __CLOTH_H__
#include "fem/World.h"
#include "fem/Mesh/MeshHeader.h"
#include "fem/Constraint/ConstraintHeader.h"
class Cloth 
{
public:
	Cloth();
	void Initialize(FEM::World* world);
	void SetMesh();
	FEM::Mesh* GetMesh() {return mMesh;};

	//Added for movement
	Eigen::Vector3d										RefPosition;
	FEM::Constraint*									PosConstraint;

	
private:
	std::vector<FEM::Constraint*>						mConstraints;
	FEM::Mesh*											mMesh;

	double 												mStretchingStiffness;
	double 												mBendingStiffness;
};

class BodyModel
{
public:
	BodyModel() = delete;
	BodyModel(	const double &length, const size_t &n_vert_fragments,
				const double &inner_radius, const double &outer_radius, const size_t &n_circ_fragments);
	void Initialize(FEM::World* world);
	//void SetMesh();
	//FEM::Mesh* GetMesh() {return mMesh;};

	//Added for movement
	size_t												RefID;
	Eigen::Vector3d										RefPosition;
	FEM::Constraint*									RefPosConstraint;

	size_t												EndID;
	Eigen::Vector3d										EndPosition;
	FEM::Constraint*									EndPosConstraint;

private:
	std::vector<FEM::Constraint*>						mConstraints;
	//FEM::Mesh*											mMesh;

	std::vector<Eigen::Vector3d>						mParticles;
	std::vector<Eigen::Vector2d>						mSprings;

	double 												mStretchingStiffness_soft;
	double 												mBendingStiffness_soft;

	double 												mStretchingStiffness_hard;
	double 												mBendingStiffness_hard;

/*
	double												length;
	size_t												n_vert_fragments;

	double												inner_radius;
*/
	
};

#endif
