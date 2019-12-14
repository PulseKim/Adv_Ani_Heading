#include "Cloth.h"
#include <iostream>
using namespace FEM;

Cloth::Cloth(	const Eigen::Vector3d &Pelvis,
				const Eigen::Vector3d &Neck,
				const Eigen::Vector3d &LShoulder,
				const Eigen::Vector3d &LAnkle,
				const Eigen::Vector3d &RShoulder,
				const Eigen::Vector3d &RAnkle,
				const Eigen::Vector3d &SkirtRoot)
{

	mConstraints.clear();
	mParticles.clear();
	FixedParams.clear();

	//Torso skeleton
	for(size_t i = 0 ; i <= n_long_fragments ; i++)
	{
		//Interpolation and add constraint
		Eigen::Vector3d position = (Pelvis * (n_long_fragments - i) + Neck * i) / n_long_fragments;
		mParticles.push_back(position);

		FEM::Constraint *PosConstraint = new FEM::AttachmentConstraint(500000, i, position);
		ConstraintParam NewAttachmentParam{	.ID				= i,
											.Position		= position,
											.PosConstraint	= PosConstraint};
		BodyParams.push_back(NewAttachmentParam);

		if(!i || i == n_long_fragments)
		{
				mConstraints.push_back(PosConstraint);
				FixedParams.push_back(NewAttachmentParam);

				if(!i)
				{
					continue;
				}
		}

		//Connect with previous one
		mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
															i - 1,
															i,
															(mParticles[i - 1] - mParticles[i]).norm()));
	}

	//Left shoulder
	for(size_t i = 1 ; i <= n_short_fragments ; i++)
	{
		Eigen::Vector3d position = (Neck * (n_short_fragments - i) + LShoulder * i) / n_short_fragments;
		mParticles.push_back((Neck * (n_short_fragments - i) + LShoulder * i) / n_short_fragments);

		ConstraintParam NewAttachmentParam{	.ID				= 20 + i,
											.Position		= position,
											.PosConstraint	= nullptr};
		LeftShoulderParams.push_back(NewAttachmentParam);

		mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
															20 + i - 1,
															20 + i,
															(mParticles[20 + i - 1] - mParticles[20 + i]).norm()));


	}

	FEM::Constraint *PosConstraint = new FEM::AttachmentConstraint(500000, 30, LShoulder);
	mConstraints.push_back(PosConstraint);

	ConstraintParam NewAttachmentParamLS{	.ID				= 30,
											.Position		= Eigen::Vector3d(0, 0, 0),
											.PosConstraint	= PosConstraint};
	FixedParams.push_back(NewAttachmentParamLS);


	//Upper left arm
	for(size_t i = 1 ; i <= n_short_fragments ; i++)
	{
		//Interpolation and add constraint
		Eigen::Vector3d position = (LShoulder * (n_short_fragments - i) + LAnkle * i) / n_short_fragments;
		mParticles.push_back((LShoulder * (n_short_fragments - i) + LAnkle * i) / n_short_fragments);

		ConstraintParam NewAttachmentParam{	.ID				= 30 + i,
											.Position		= position,
											.PosConstraint	= nullptr};
		LeftArmParams.push_back(NewAttachmentParam);

		mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
															30 + i - 1,
															30 + i,
															(mParticles[30 + i - 1] - mParticles[30 + i]).norm()));
	}
	PosConstraint = new FEM::AttachmentConstraint(500000, 40, LAnkle);
	mConstraints.push_back(PosConstraint);

	ConstraintParam NewAttachmentParamLA{	.ID				= 40,
											.Position		= Eigen::Vector3d(0, 0, 0),
											.PosConstraint	= PosConstraint};
	FixedParams.push_back(NewAttachmentParamLA);

	//Right shoulder
	for(size_t i = 1 ; i <= n_short_fragments ; i++)
	{
		Eigen::Vector3d position = (Neck * (n_short_fragments - i) + RShoulder * i) / n_short_fragments;
		mParticles.push_back((Neck * (n_short_fragments - i) + RShoulder * i) / n_short_fragments);

		ConstraintParam NewAttachmentParam{	.ID				= 40 + i,
											.Position		= position,
											.PosConstraint	= nullptr};
		RightShoulderParams.push_back(NewAttachmentParam);

		if(i == 1)
		{
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
																20,
																41,
																(mParticles[20] - mParticles[41]).norm()));
			continue;
		}

		mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
															40 + i - 1,
															40 + i,
															(mParticles[40 + i - 1] - mParticles[40 + i]).norm()));
	}

	//Insert to FixedParams
	PosConstraint = new FEM::AttachmentConstraint(500000, 50, RShoulder);
	mConstraints.push_back(PosConstraint);

	ConstraintParam NewAttachmentParamRS{	.ID				= 50,
											.Position		= Eigen::Vector3d(0, 0, 0),
											.PosConstraint	= PosConstraint};
	FixedParams.push_back(NewAttachmentParamRS);

	//Upper right arm
	for(size_t i = 1 ; i <= n_short_fragments ; i++)
	{
		//Interpolation and add constraint
		Eigen::Vector3d position = (RShoulder * (n_short_fragments - i) + RAnkle * i) / n_short_fragments;
		mParticles.push_back((RShoulder * (n_short_fragments - i) + RAnkle * i) / n_short_fragments);
		ConstraintParam NewAttachmentParam{	.ID				= 50 + i,
											.Position		= position,
											.PosConstraint	= nullptr};
		RightArmParams.push_back(NewAttachmentParam);

		mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
															50 + i - 1,
															50 + i,
															(mParticles[50 + i - 1] - mParticles[50 + i]).norm()));
	}
	PosConstraint = new FEM::AttachmentConstraint(500000, 60, RAnkle);
	mConstraints.push_back(PosConstraint);

	ConstraintParam NewAttachmentParamRA{	.ID				= 60,
											.Position		= Eigen::Vector3d(0, 0, 0),
											.PosConstraint	= PosConstraint};
	FixedParams.push_back(NewAttachmentParamRA);
	
	//Pelvis to skirt root
	for(size_t i = 1 ; i <= n_long_fragments ; i++)
	{
		//Interpolation and add constraint
		Eigen::Vector3d position = (Pelvis * (n_long_fragments - i) + SkirtRoot * i) / n_long_fragments;
		mParticles.push_back((Pelvis * (n_long_fragments - i) + SkirtRoot * i) / n_long_fragments);

		ConstraintParam NewAttachmentParam{	.ID				= 60 + i,
											.Position		= position,
											.PosConstraint	= PosConstraint};

		SkirtRootParams.push_back(NewAttachmentParam);

		if(i == 1)
		{
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
																0,
																61,
																(mParticles[61] - mParticles[0]).norm()));
			continue;
		}

		mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
															60 + i - 1,
															60 + i,
															(mParticles[60 + i - 1] - mParticles[60 + i]).norm()));

	}
	PosConstraint = new FEM::AttachmentConstraint(500000, 80, SkirtRoot);
	mConstraints.push_back(PosConstraint);

	ConstraintParam NewAttachmentParamSk{	.ID				= 80,
											.Position		= Eigen::Vector3d(0, 0, 0),
											.PosConstraint	= PosConstraint};
	FixedParams.push_back(NewAttachmentParamSk);

	std::cout << mParticles.size() << std::endl;
	std::cout << FixedParams.size() << std::endl;

	//Create actual skirt vertices and constraints
	//Particle 81-400

	size_t ParticleOffset = 81;
	float SkirtRadius = 2.1f;
	for(auto it = SkirtRootParams.begin() ; it != SkirtRootParams.end() ; it++)
	{

		Eigen::Vector3d ref_pos = it -> Position;
		for(size_t i = 0 ; i < n_circ_fragments ; i++)
		{
			Eigen::Vector3d NewPos =	it -> Position
										+ Eigen::Vector3d(SkirtRadius * cos((2 * 3.141591f * i) / n_circ_fragments),
														0, SkirtRadius * sin((2 * 3.141591f * i) / n_circ_fragments));
			mParticles.push_back(NewPos);

			//In-out connection
			double stiffness = it == SkirtRootParams.begin() ? mStretchingStiffness_hard : mStretchingStiffness_soft;

			mConstraints.push_back(new FEM::SpringConstraint(	stiffness,
																ParticleOffset + i,
																it -> ID,
																SkirtRadius));
			//Rim
			mConstraints.push_back(new FEM::SpringConstraint(	stiffness,
																ParticleOffset + i,
																ParticleOffset + ((i + 1) % n_circ_fragments),
																(SkirtRadius * 2 * 3.141591f) / n_circ_fragments));
		}

		if(it == SkirtRootParams.begin())
		{
			ParticleOffset += n_circ_fragments;
			SkirtRadius += 0.1f;
			continue;
		}

		//Vertical interconnection
		for(size_t i = 0 ; i < n_circ_fragments ; i++)
		{
			size_t i0 = ParticleOffset + i;
			size_t i1 = i0 - n_circ_fragments;

			//Vertical
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));

			//TODO: Remove X-shaped and take video (Later)
			i0 = ParticleOffset - n_circ_fragments + ((i + 1) % n_circ_fragments);
			i1 = ParticleOffset + i;
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));

			i0 = ParticleOffset - n_circ_fragments + i;
			i1 = ParticleOffset + ((i + 1) % n_circ_fragments);
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));
		}

		ParticleOffset += n_circ_fragments;
		SkirtRadius += 0.1f;
	}


	//Torso vertices(401 - 640, 641 - 720)

	SkirtRadius = 2.0f;
	size_t n_rings = 0;
	for(auto it = BodyParams.begin() ; it + 1 != BodyParams.end() ; it++)
	{

		Eigen::Vector3d ref_pos = it -> Position;
		for(size_t i = 0 ; i < n_circ_fragments ; i++)
		{
			Eigen::Vector3d NewPos =	it -> Position
										+ Eigen::Vector3d(SkirtRadius * cos((2 * 3.141591f * i) / n_circ_fragments),
														0, SkirtRadius * sin((2 * 3.141591f * i) / n_circ_fragments));
			mParticles.push_back(NewPos);

			double stiffness = it == BodyParams.begin() ? mStretchingStiffness_hard : mStretchingStiffness_soft;

			//In-out connection
			if(++n_rings < 15)
			{
					mConstraints.push_back(new FEM::SpringConstraint(	stiffness,
																		ParticleOffset + i,
																		it -> ID,
																		SkirtRadius));
			}

			//Rim
			mConstraints.push_back(new FEM::SpringConstraint(	stiffness,
																ParticleOffset + i,
																ParticleOffset + ((i + 1) % n_circ_fragments),
																(SkirtRadius * 2 * 3.141591f) / n_circ_fragments));
		}

		if(it == SkirtRootParams.begin())
		{
			ParticleOffset += n_circ_fragments;
			continue;
		}

		//Vertical interconnection
		for(size_t i = 0 ; i < n_circ_fragments ; i++)
		{
			size_t i0 = ParticleOffset + i;
			size_t i1 = i0 - n_circ_fragments;

			//Vertical
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));

			//TODO: Remove X-shaped and take video (Later)
			i0 = ParticleOffset - n_circ_fragments + ((i + 1) % n_circ_fragments);
			i1 = ParticleOffset + i;
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));

			i0 = ParticleOffset - n_circ_fragments + i;
			i1 = ParticleOffset + ((i + 1) % n_circ_fragments);
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));
		}

		ParticleOffset += n_circ_fragments;
	}
	std::cout << ParticleOffset << std::endl;

	//Left shoulder vertices: 721-810
	double ShoulderRadius = 0.5f;

	for(auto it = LeftShoulderParams.begin() ; it != LeftShoulderParams.end() ; it++)
	{
		Eigen::Vector3d ref_pos = it -> Position;
		for(size_t i = 0 ; i < ((n_circ_fragments / 2) + 1) ; i++)
		{
			Eigen::Vector3d NewPos =	it -> Position
										+ Eigen::Vector3d(	0, 
															ShoulderRadius * sin((2 * 3.141591f * i) / n_circ_fragments),
															-ShoulderRadius * cos((2 * 3.141591f * i) / n_circ_fragments));
			mParticles.push_back(NewPos);

			//Rim
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
																it -> ID,
																ParticleOffset + i,
																ShoulderRadius));

			if(i == n_circ_fragments / 2)
			{
				break;
			}

			//Internal connection
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
																ParticleOffset + i,
																ParticleOffset + i + 1,
																(ShoulderRadius * 2 * 3.141591f) / n_circ_fragments));
		}

		if(it + 1 == LeftShoulderParams.begin())
		{
			ParticleOffset += ((n_circ_fragments / 2) + 1);
			continue;
		}
		
		for(size_t i = 0 ; i < ((n_circ_fragments / 2) + 1) ; i++)
		{
			size_t i0 = ParticleOffset + i;
			size_t i1 = ParticleOffset + i - ((n_circ_fragments) / 2) + 1;

			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));
		}

		ParticleOffset += ((n_circ_fragments / 2) + 1);
	}


	std::cout << ParticleOffset << std::endl;

	//Right shoulder vertices: 811-900
	for(auto it = RightShoulderParams.begin() ; it != RightShoulderParams.end() ; it++)
	{
		Eigen::Vector3d ref_pos = it -> Position;
		for(size_t i = 0 ; i < ((n_circ_fragments / 2) + 1) ; i++)
		{
			Eigen::Vector3d NewPos =	it -> Position
										+ Eigen::Vector3d(	0, 
															ShoulderRadius * sin((2 * 3.141591f * i) / n_circ_fragments),
															-ShoulderRadius * cos((2 * 3.141591f * i) / n_circ_fragments));
			mParticles.push_back(NewPos);

			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
																it -> ID,
																ParticleOffset + i,
																ShoulderRadius));

			if(i == n_circ_fragments / 2)
			{
				break;
			}

			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
																ParticleOffset + i,
																ParticleOffset + i + 1,
																(ShoulderRadius * 2 * 3.141591f) / n_circ_fragments));
		}

		if(it + 1 == LeftShoulderParams.begin())
		{
			ParticleOffset += ((n_circ_fragments / 2) + 1);
			continue;
		}
		
		for(size_t i = 0 ; i < ((n_circ_fragments / 2) + 1) ; i++)
		{
			size_t i0 = ParticleOffset + i;
			size_t i1 = ParticleOffset + i - ((n_circ_fragments) / 2) + 1;

			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));
		}

		ParticleOffset += ((n_circ_fragments / 2) + 1);
	}
	std::cout << ParticleOffset << std::endl;


	//Connect: Length = ShoulderRadius * 2 * 3.141591f / n_circ_fragments

	size_t TorsoStitchLHS[12] = {706, 707, 708, 710, 711, 712, 714, 715, 716, 718, 719, 720};
	size_t TorsoStitchRHS[12] = {793, 757, 721, 811, 847, 883, 891, 855, 819, 709, 729, 801};

	for(size_t i = 0 ; i < 12 ; i++)
	{
		mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_hard,
															TorsoStitchLHS[i],
															TorsoStitchRHS[i],
															(ShoulderRadius * 2 * 3.141591f) / n_circ_fragments));
	}

	//Left arm vertices: 901-1060
	double ArmRadius = 0.6f;

	for(auto it = LeftArmParams.begin() ; it != LeftArmParams.end() ; it++)
	{
		Eigen::Vector3d ref_pos = it -> Position;
		for(size_t i = 0 ; i < n_circ_fragments ; i++)
		{
			Eigen::Vector3d NewPos =	it -> Position
										+ Eigen::Vector3d(ArmRadius * cos((2 * 3.141591f * i) / n_circ_fragments),
														0, ArmRadius * sin((2 * 3.141591f * i) / n_circ_fragments));
			mParticles.push_back(NewPos);

			//In-out connection
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft / 4,
																ParticleOffset + i,
																it -> ID,
																ArmRadius));
			//Rim
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																ParticleOffset + i,
																ParticleOffset + ((i + 1) % n_circ_fragments),
																(ArmRadius * 2 * 3.141591f) / n_circ_fragments));
		}

		if(it + 1 == LeftArmParams.begin())
		{
			ParticleOffset += n_circ_fragments;
			continue;
		}

		for(size_t i = 0 ; i < n_circ_fragments ; i++)
		{
			size_t i0 = ParticleOffset + i;
			size_t i1 = i0 - n_circ_fragments;

			//Vertical
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft / 4,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));

			//TODO: Remove X-shaped and take video (Later)
			i0 = ParticleOffset - n_circ_fragments + ((i + 1) % n_circ_fragments);
			i1 = ParticleOffset + i;
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));

			i0 = ParticleOffset - n_circ_fragments + i;
			i1 = ParticleOffset + ((i + 1) % n_circ_fragments);
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));
		}

		ParticleOffset += n_circ_fragments;
	}
	std::cout << ParticleOffset << std::endl;

	//size_t LeftArmStitchLHS[9] = {802, 803, 804, ... 810}
	size_t LeftArmStitchRHS[9] = {905, 904, 903, 902, 901, 916, 915, 914, 913};

	for(size_t i = 0 ; i < 9 ; i++)
	{
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft * 4,
																802 + i,
																LeftArmStitchRHS[i],
																(mParticles[802+i] - mParticles[LeftArmStitchRHS[i]]).norm() * 0.7));
	}


	for(auto it = RightArmParams.begin() ; it != RightArmParams.end() ; it++)
	{
		Eigen::Vector3d ref_pos = it -> Position;
		for(size_t i = 0 ; i < n_circ_fragments ; i++)
		{
			Eigen::Vector3d NewPos =	it -> Position
										+ Eigen::Vector3d(ArmRadius * cos((2 * 3.141591f * i) / n_circ_fragments),
														0, ArmRadius * sin((2 * 3.141591f * i) / n_circ_fragments));
			mParticles.push_back(NewPos);

			//In-out connection
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft / 4,
																ParticleOffset + i,
																it -> ID,
																ArmRadius));
			//Rim
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																ParticleOffset + i,
																ParticleOffset + ((i + 1) % n_circ_fragments),
																(ArmRadius * 2 * 3.141591f) / n_circ_fragments));
		}

		if(it + 1 == RightArmParams.begin())
		{
			ParticleOffset += n_circ_fragments;
			continue;
		}

		for(size_t i = 0 ; i < n_circ_fragments ; i++)
		{
			size_t i0 = ParticleOffset + i;
			size_t i1 = i0 - n_circ_fragments;

			//Vertical
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft / 4,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));

			//TODO: Remove X-shaped and take video (Later)
			i0 = ParticleOffset - n_circ_fragments + ((i + 1) % n_circ_fragments);
			i1 = ParticleOffset + i;
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));

			i0 = ParticleOffset - n_circ_fragments + i;
			i1 = ParticleOffset + ((i + 1) % n_circ_fragments);
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft,
																i0,
																i1,
																(mParticles[i1] - mParticles[i0]).norm()));
		}

		ParticleOffset += n_circ_fragments;
	}

	for(size_t i = 0 ; i < 9 ; i++)
	{
			mConstraints.push_back(new FEM::SpringConstraint(	mStretchingStiffness_soft * 4,
																892 + i,
																1065 + i,
																(mParticles[892+i] - mParticles[1065+i]).norm() * 0.7));
	}


	//Create world.
	mSoftWorld = new FEM::World(
		FEM::IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
		FEM::OptimizationMethod::OPTIMIZATION_METHOD_NEWTON,
		FEM::LinearSolveType::SOLVER_TYPE_LDLT,
		1.0/1000.0,										//time_step
		100, 											//max_iteration	
		0.99											//damping_coeff
		);

	//Pass vertices to world
	Eigen::VectorXd p(mParticles.size()*3);
	for(int i =0;i<mParticles.size();i++)
		p.block<3,1>(i*3,0) = mParticles[i];

	mSoftWorld->AddBody(p,mConstraints,1.0);


	//Pass constraints to world
	for(auto& c: mConstraints) {
		mSoftWorld->AddConstraint(c);
	}

	//Initialize our world
	mSoftWorld->Initialize();
}


void Cloth::SetPosition(	const Eigen::Vector3d &Pelvis,
							const Eigen::Vector3d &Neck,
							const Eigen::Vector3d &LShoulder,
							const Eigen::Vector3d &LAnkle,
							const Eigen::Vector3d &RShoulder,
							const Eigen::Vector3d &RAnkle,
							const Eigen::Vector3d &SkirtRoot)
{
	for(auto it = FixedParams.begin() ; it != FixedParams.end() ; it++)
	{
		mSoftWorld->RemoveConstraint(it -> PosConstraint);
		delete it -> PosConstraint;
	}

	FixedParams[0].Position = Pelvis;
	FixedParams[1].Position = Neck;
	FixedParams[2].Position = LShoulder;
	FixedParams[3].Position = LAnkle;
	FixedParams[4].Position = RShoulder;
	FixedParams[5].Position = RAnkle;
	FixedParams[6].Position = SkirtRoot;

	for(auto it = FixedParams.begin() ; it != FixedParams.end() ; it++)
	{
		it -> PosConstraint = new FEM::AttachmentConstraint(500000, it -> ID, it -> Position);
		mSoftWorld -> AddConstraint(it -> PosConstraint);
	}
}

void Cloth::TimeStep()
{
	mSoftWorld->TimeStepping();
}

FEM::World *Cloth::GetSoftWorld()
{
	return this -> mSoftWorld;
}

const std::vector<Eigen::Vector3d> Cloth::getVertices()
{
	const Eigen::VectorXd &particles = this->mSoftWorld->GetPositions();

	std::vector<Eigen::Vector3d> ret;

	//Lower skirt: 81-384
	size_t ParticleOffset = 81;
	for(size_t i = 0 ; i < n_long_fragments - 1 ; i++)
	{
		for(size_t j = 0 ; j < n_circ_fragments - 1 ; j++)
		{
			ret.emplace_back(	particles[ParticleOffset * 3 + 0],
								particles[ParticleOffset * 3 + 1],
								particles[ParticleOffset * 3 + 2]);	//81

			ret.emplace_back(	particles[(ParticleOffset + 1) * 3 + 0],
								particles[(ParticleOffset + 1) * 3 + 1],
								particles[(ParticleOffset + 1) * 3 + 2]);	//82

			ret.emplace_back(	particles[(ParticleOffset + n_circ_fragments + 1) * 3 + 0],
								particles[(ParticleOffset + n_circ_fragments + 1) * 3 + 1],
								particles[(ParticleOffset + n_circ_fragments + 1) * 3 + 2]);	//98

			ret.emplace_back(	particles[(ParticleOffset + n_circ_fragments) * 3 + 0],
								particles[(ParticleOffset + n_circ_fragments) * 3 + 1],
								particles[(ParticleOffset + n_circ_fragments) * 3 + 2]);		//97

			ParticleOffset++;

		}

		ret.emplace_back(	particles[ParticleOffset * 3 + 0],
							particles[ParticleOffset * 3 + 1],
							particles[ParticleOffset * 3 + 2]);

		ret.emplace_back(	particles[(ParticleOffset - n_circ_fragments + 1) * 3 + 0],
							particles[(ParticleOffset - n_circ_fragments + 1) * 3 + 1],
							particles[(ParticleOffset - n_circ_fragments + 1) * 3 + 2]);

		ret.emplace_back(	particles[(ParticleOffset + 1) * 3 + 0],
							particles[(ParticleOffset + 1) * 3 + 1],
							particles[(ParticleOffset + 1) * 3 + 2]);

		ret.emplace_back(	particles[(ParticleOffset + n_circ_fragments) * 3 + 0],
							particles[(ParticleOffset + n_circ_fragments) * 3 + 1],
							particles[(ParticleOffset + n_circ_fragments) * 3 + 2]);

		ParticleOffset++;
		
	}


	//Torso: 401-720
	ParticleOffset = 401;
	for(size_t i = 0 ; i < n_long_fragments ; i++)
	{
		for(size_t j = 0 ; j < n_circ_fragments - 1 ; j++)
		{
			ret.emplace_back(	particles[ParticleOffset * 3 + 0],
								particles[ParticleOffset * 3 + 1],
								particles[ParticleOffset * 3 + 2]);	//81

			ret.emplace_back(	particles[(ParticleOffset + 1) * 3 + 0],
								particles[(ParticleOffset + 1) * 3 + 1],
								particles[(ParticleOffset + 1) * 3 + 2]);	//82

			ret.emplace_back(	particles[(ParticleOffset + n_circ_fragments + 1) * 3 + 0],
								particles[(ParticleOffset + n_circ_fragments + 1) * 3 + 1],
								particles[(ParticleOffset + n_circ_fragments + 1) * 3 + 2]);	//98

			ret.emplace_back(	particles[(ParticleOffset + n_circ_fragments) * 3 + 0],
								particles[(ParticleOffset + n_circ_fragments) * 3 + 1],
								particles[(ParticleOffset + n_circ_fragments) * 3 + 2]);		//97

			ParticleOffset++;

		}

		ret.emplace_back(	particles[ParticleOffset * 3 + 0],
							particles[ParticleOffset * 3 + 1],
							particles[ParticleOffset * 3 + 2]);

		ret.emplace_back(	particles[(ParticleOffset - n_circ_fragments + 1) * 3 + 0],
							particles[(ParticleOffset - n_circ_fragments + 1) * 3 + 1],
							particles[(ParticleOffset - n_circ_fragments + 1) * 3 + 2]);

		ret.emplace_back(	particles[(ParticleOffset + 1) * 3 + 0],
							particles[(ParticleOffset + 1) * 3 + 1],
							particles[(ParticleOffset + 1) * 3 + 2]);

		ret.emplace_back(	particles[(ParticleOffset + n_circ_fragments) * 3 + 0],
							particles[(ParticleOffset + n_circ_fragments) * 3 + 1],
							particles[(ParticleOffset + n_circ_fragments) * 3 + 2]);

		ParticleOffset++;
		
	}

	return ret;

}
