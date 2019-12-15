#ifndef __CLOTH_H__
#define __CLOTH_H__
#include "fem/World.h"
#include "fem/Mesh/MeshHeader.h"
#include "fem/Constraint/ConstraintHeader.h"

class Cloth
{
public:
	Cloth() = delete;
	Cloth(	const Eigen::Vector3d &Pelvis,
			const Eigen::Vector3d &Neck,
			const Eigen::Vector3d &LShoulder,
			const Eigen::Vector3d &LAnkle,
			const Eigen::Vector3d &RShoulder,
			const Eigen::Vector3d &RAnkle,
			const Eigen::Vector3d &SkirtRoot);
	~Cloth();

	void SetPosition(	const Eigen::Vector3d &Pelvis,
						const Eigen::Vector3d &Neck,
						const Eigen::Vector3d &LShoulder,
						const Eigen::Vector3d &LAnkle,
						const Eigen::Vector3d &RShoulder,
						const Eigen::Vector3d &RAnkle,
						const Eigen::Vector3d &SkirtRoot);

	void TimeStep();
	FEM::World* GetSoftWorld();
	const std::vector<Eigen::Vector3d> getVertices();


private:
	//TBD- (Jeonghun)

	//Particle simulation world
	FEM::World*											mSoftWorld;

	//Vertecies, springs and constraints
	std::vector<FEM::Constraint*>						mConstraints;
	std::vector<Eigen::Vector3d>						mParticles;

	//Created for movement
	typedef struct
	{
		size_t											ID;
		Eigen::Vector3d									Position;
		FEM::Constraint*								PosConstraint;
	}ConstraintParam;

	//Endoskeleton(?) nodes
	std::vector<ConstraintParam>						LeftArmParams;
	std::vector<ConstraintParam>						LeftShoulderParams;

	std::vector<ConstraintParam>						RightArmParams;
	std::vector<ConstraintParam>						RightShoulderParams;

	std::vector<ConstraintParam>						BodyParams;
	std::vector<ConstraintParam>						SkirtRootParams;

	std::vector<ConstraintParam>						FixedParams;
	
	//Simulation timestep
	const double										TimeStepSize = 0.4f;

	//Spring stiffness parameters
	const double										mStretchingStiffness_soft = 100.0f;
	const double										mBendingStiffness_soft = 100.0f;

	const double										mStretchingStiffness_hard = 100000.0f;
	const double										mBendingStiffness_hard = 100000.0f;

	//Size parameters
	const double										TorsoRadius = 0.12f;
	const double										ShoulderRadius = 0.02f;
	const double										ArmRadius = 0.05f;
	const double										SkirtStepping = 0.02f;

	//Number of fragments
	static const size_t									n_circ_fragments = 16;
	static const size_t									n_long_fragments = 20;
	static const size_t									n_short_fragments = 10;

};


#endif
