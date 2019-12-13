#include "Cloth.h"
#include <iostream>
using namespace FEM;

Cloth::
Cloth()
:mMesh(),mStretchingStiffness(1E2),mBendingStiffness(20.0)
{

}
void
Cloth::
Initialize(FEM::World* world)
{
	Eigen::Affine3d T=Eigen::Affine3d::Identity();
	mMesh = new GridMesh(10,10,10.0,10.0,Eigen::Vector3d(-1.0,1.0,0),T);	

	const auto& particles = mMesh->GetParticles();
	const auto& springs = mMesh->GetSprings();

	int idx = 0;
	for(const auto& spr : springs) 
	{
		int i0,i1; 
		Eigen::Vector3d p0,p1;

		i0 = spr[0];
		i1 = spr[1];
		p0 = particles[i0];
		p1 = particles[i1];

		double l0 = (p0-p1).norm();

		mConstraints.push_back(new SpringConstraint(mStretchingStiffness,i0,i1,l0));
		idx +=1;
	}

	this->RefPosition = particles[1];
	this->PosConstraint = new FEM::AttachmentConstraint(500000,1,this->RefPosition);
	world->AddConstraint(this->PosConstraint);

	//world->AddConstraint(new FEM::AttachmentConstraint(500000,1,particles[1]));


	//world->AddConstraint(new FEM::AttachmentConstraint(500000,10*10-1,particles[10*10-1]));

	//world->AddConstraint(new FEM::AttachmentConstraint(500000,1*10-1,particles[1*10-1]));
	// world->AddConstraint(new FEM::AttachmentConstraint(500000,3*10-1,particles[3*10-1]));
	// world->AddConstraint(new FEM::AttachmentConstraint(500000,5*10-1,particles[5*10-1]));
	// world->AddConstraint(new FEM::AttachmentConstraint(500000,7*10-1,particles[7*10-1]));
	// world->AddConstraint(new FEM::AttachmentConstraint(500000,10*10-1,particles[10*10-1]));

	Eigen::VectorXd p(particles.size()*3);
	for(int i =0;i<particles.size();i++)
		p.block<3,1>(i*3,0) = particles[i];

	world->AddBody(p,mConstraints,1.0);

	for(auto& c: mConstraints) {
		world->AddConstraint(c);
	}
}

BodyModel::
BodyModel(	const double &length, const size_t &n_vert_fragments,
			const double &inner_radius, const double &outer_radius, const size_t &n_circ_fragments)
//:mStretchingStiffness_soft(1E1),mBendingStiffness_soft(2.0),
//mStretchingStiffness_hard(1E6),mBendingStiffness_hard(100000.0)
:mStretchingStiffness_soft(1E2),mBendingStiffness_soft(2.0),
mStretchingStiffness_hard(1E6),mBendingStiffness_hard(100000.0)
{
	//Create mesh here. particles and springs

	Eigen::Affine3d T = Eigen::Affine3d::Identity();
	//size_t frag_per_circle = (n_circ_fragments * 2) + 1;

	size_t frag_per_circle = (n_circ_fragments) + 1;

	mParticles.clear();
	for(size_t i = 0 ; i <= n_vert_fragments ; i++)
	{
		double x_offset = (length * i) / n_vert_fragments;
		//Eigen::Vector3d vert_pos(x_offset, 0, 0);
		//mParticles.push_back(vert_pos);

		mParticles.emplace_back(x_offset, 0, 0);
		/*
		std::cout	<< (*(mParticles.rbegin()))[0] << ", "
					<< (*(mParticles.rbegin()))[1] << ", "
					<< (*(mParticles.rbegin()))[2] << std::endl;
					*/

		//Internal circle
		for(size_t j = 0 ; j < n_circ_fragments ; j++)
		{
			mParticles.emplace_back(x_offset,
									inner_radius * cos((((double)j) * 2.0f * 3.141591f) / n_circ_fragments),
									inner_radius * sin((((double)j) * 2.0f * 3.141591f) / n_circ_fragments));
		}

/*
		//External circle
		for(size_t j = 0 ; j < n_circ_fragments ; j++)
		{
			mParticles.emplace_back(x_offset,
									outer_radius * cos((((double)j) * 2.0f * 3.141591f) / n_circ_fragments),
									outer_radius * sin((((double)j) * 2.0f * 3.141591f) / n_circ_fragments));
		}
*/
	}
	
	//Ring structure
	for(size_t i = 0 ; i <= n_vert_fragments ; i++)
	{
		for(size_t j = 0 ; j < n_circ_fragments ; j++)
		{
			//Center to inner rim
			mConstraints.push_back(new SpringConstraint(mStretchingStiffness_hard,
														(i * frag_per_circle),			//Center point
														(i * frag_per_circle) + 1 + j,	//Each point of inner rim
														inner_radius));
			
			//Inner rim
			mConstraints.push_back(new SpringConstraint(mStretchingStiffness_soft,
														(i * frag_per_circle) + 1 + j,
														(i * frag_per_circle) + 1 + ((j + 1) % n_circ_fragments),
														(inner_radius * 2.0f * 3.141591f) / n_circ_fragments));
/*
			//Inner rim to outer rim
			mConstraints.push_back(new SpringConstraint(mStretchingStiffness_soft,
														(i * frag_per_circle) + 1 + j,
														(i * frag_per_circle) + 1 + j + n_circ_fragments,
														outer_radius - inner_radius));

			//Outer rim
			mConstraints.push_back(new SpringConstraint(mStretchingStiffness_soft,
														(i * frag_per_circle) + (n_circ_fragments + 1) + j,
														(i * frag_per_circle) + (n_circ_fragments + 1) + ((j + 1) % n_circ_fragments),
														(outer_radius * 2.0f * 3.141591f) / n_circ_fragments));
*/
		}
	}

	//Internal structures
	for(size_t i = 0 ; i < n_vert_fragments ; i++)
	{
		//Set spring constraints for central axis
		mConstraints.push_back(new SpringConstraint(mStretchingStiffness_hard,
													i * frag_per_circle,
													(i + 1) * frag_per_circle,
													length / n_vert_fragments));

		for(size_t j = 1 ; j <= n_circ_fragments ; j++)
		{
				//Inner rim interconnect
				mConstraints.push_back(new SpringConstraint(mStretchingStiffness_soft,
															j + (i * frag_per_circle),
															j + ((i + 1) * frag_per_circle),
															length / n_vert_fragments));

/*
				//Outer rim interconnect
				mConstraints.push_back(new SpringConstraint(mStretchingStiffness_soft,
															j + n_circ_fragments + (i * frag_per_circle),
															j + n_circ_fragments + ((i + 1) * frag_per_circle),
															length / n_vert_fragments));
*/
		}



	}


	//Set starting point and end point-
	RefID = 0;
	RefPosition = Eigen::Vector3d(0, 0, 0);
	RefPosConstraint = new FEM::AttachmentConstraint(500000, RefID, this -> RefPosition);

	EndID = frag_per_circle * (n_vert_fragments);
	EndPosition = Eigen::Vector3d(length, 0, 0);
	EndPosConstraint = new FEM::AttachmentConstraint(500000, EndID, this -> EndPosition);

}

void
BodyModel::
Initialize(FEM::World* world)

{
	
	//Add contraints to world
	Eigen::VectorXd p(mParticles.size()*3);
	for(int i =0;i<mParticles.size();i++)
		p.block<3,1>(i*3,0) = mParticles[i];

	world->AddBody(p,mConstraints,1.0);

	for(auto& c: mConstraints) {
		world->AddConstraint(c);
	}

	world->AddConstraint(RefPosConstraint);
	world->AddConstraint(EndPosConstraint);

}
