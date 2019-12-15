#include "MyWindow.h"

MyWindow::MyWindow(const WorldPtr& world) : SimWindow()
{
	PD_flag = true;
	bvh_flag = false;
	this->setWorld(world);
	this->initParameters();
	this->initWindowSetting();
	std::string file_name = "16_01_jump.bvh";
	mbvh = std::make_unique<bvh>(file_name, mHubo, mWorld->getTimeStep());
	mMotionGraph = std::make_unique<MotionGraph>(mHubo, mWorld->getTimeStep());
	mBallGenerator = std::make_unique<Throw>(mHubo, mWorld->getTimeStep());
	mCartoonFlag = false;
	this->water_flag = false;


	Eigen::Vector3d skirtRoot_dir = (getJointTransform("tibial") + getJointTransform("tibiar"))/2 - getJointTransform("torso");
	Eigen::Vector3d skirtRoot = skirtRoot_dir/skirtRoot_dir.norm()*0.5 + getJointTransform("torso");

	std::cout << getJointTransform("torso").transpose() << std::endl;
	std::cout << getJointTransform("head").transpose() << std::endl;
	std::cout << getJointTransform("upperarml").transpose() << std::endl;
	std::cout << getJointTransform("forearml").transpose() << std::endl;
	std::cout << getJointTransform("upperarmr").transpose() << std::endl;
	std::cout << getJointTransform("forearmr").transpose() << std::endl;
	std::cout << skirtRoot.transpose() << std::endl;



	mCloth = std::make_unique<Cloth>(getJointTransform("torso"),
						getJointTransform("head"),
						getJointTransform("upperarml"),
						getJointTransform("forearml"),
						getJointTransform("upperarmr"),
						getJointTransform("forearmr"),
						skirtRoot
	);

	std::cout << "pass" << std::endl;

}


//Initiation functions that initailize Parameter, Skeleton and world.
void MyWindow::initParameters()
{
	mWorld->setGravity(Eigen::Vector3d(0, -9.81, 0));
	cnt =0;
	this->motion_count = 0;
}


void MyWindow::loadHuboFloor()
{
  // Create the world with a skeleton
	std::string build_dir = this->GetCurrentWorkingDir();
	WorldPtr world_temp = SkelParser::readWorld(build_dir + "/../data/hubo.skel");
	assert(world_temp != nullptr);

	SkeletonPtr biped = world_temp->getSkeleton("hubo");
	SkeletonPtr floor = world_temp->getSkeleton("ground skeleton");

	// Set joint limits
	for(std::size_t i = 0; i < biped->getNumJoints(); ++i)
	  biped->getJoint(i)->setPositionLimitEnforced(true);
	  
	// Enable self collision check but ignore adjacent bodies
	biped->disableAdjacentBodyCheck();
	mHubo = biped;
	mFloor = floor;
}

void MyWindow::initWindowSetting()
{	 
	mWorld->getConstraintSolver()->setCollisionDetector(
    	  dart::collision::BulletCollisionDetector::create());

	if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet"))
  	{
    	mWorld->getConstraintSolver()->setCollisionDetector(
    	  dart::collision::BulletCollisionDetector::create());
  	}

	this->initSkeleton();
	this->setSkeleton();
	this->addSkeleton();
	mController = std::make_unique<Controller>(mHubo);
}

void MyWindow::initSkeleton()
{
	this->loadHuboFloor();
	mBall = Skeleton::create("ball");
	float ball_rad = 0.11;
	SkelGen skel;
	skel.freeSphere(mBall, "ball", ball_rad, Eigen::Vector3d(0,0, 0), 1.0, dart::Color::Red());
}

void MyWindow::setSkeleton()
{
	Eigen::VectorXd default_pose = mHubo->getPositions();
	default_pose[4] = 0.745;
	mHubo->setPositions(default_pose);
	mBall->setPosition(3, 0.0);
	mBall->setPosition(4, 0.5);
	mBall->setPosition(5, 30.0);
	// Visual Aspect
	auto visualShapenodes = mFloor->getBodyNode(0)->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(dart::Color::White(0.50));
}

void MyWindow::addSkeleton()
{
	mFloor->getBodyNode(0)->setFrictionCoeff(0.5);
	mWorld->addSkeleton(mHubo);
	mWorld->addSkeleton(mFloor);
	mWorld->addSkeleton(mBall);
}

std::string MyWindow::GetCurrentWorkingDir() 
{
  char buff[FILENAME_MAX];
  GetCurrentDir( buff, FILENAME_MAX );
  std::string current_working_dir(buff);
  return current_working_dir;
}

void MyWindow::throw_normal_ball()
{
	SkeletonPtr original_skel = mWorld->getSkeleton("ball");
	mWorld->removeSkeleton(original_skel);
	mBall = Skeleton::create("ball");
	float ball_rad = 0.11;
	SkelGen skel;
	// skel.freeSphere(mBall, "ball", ball_rad, Eigen::Vector3d(0,0, 0), 0.1, dart::Color::Blue());
	skel.freeSoftSphere(mBall, "ball", ball_rad, Eigen::Vector3d(0,0, 0), 0.1, dart::Color::Red());
	auto last_motion =  mCurrentMotionblender->root_aligned_pose(mMotions[mMotions.size()-1]);
	mBall->setPosition(3, last_motion[3]  + mBallGenerator->mInitPos[0]);
	mBall->setPosition(4, mBallGenerator->mInitPos[1]);
	mBall->setPosition(5, last_motion[5] + mBallGenerator->mInitPos[2]);
	mBall->setVelocity(3, mBallGenerator->mInitVel[0]);
	mBall->setVelocity(4, mBallGenerator->mInitVel[1]);
	mBall->setVelocity(5, mBallGenerator->mInitVel[2]);
	mWorld->addSkeleton(mBall);	

	this->water_flag = false;
}
void MyWindow::throw_water_ball()
{
	SkeletonPtr original_skel = mWorld->getSkeleton("ball");
	mWorld->removeSkeleton(original_skel);
	mBall = Skeleton::create("ball");
	float ball_rad = 0.11;
	SkelGen skel;
	skel.freeSphere(mBall, "ball", ball_rad, Eigen::Vector3d(0,0, 0), 1.0, dart::Color::Red());
	auto last_motion =  mCurrentMotionblender->root_aligned_pose(mMotions[mMotions.size()-1]);
	mBall->setPosition(3, last_motion[3]  + mBallGenerator->mInitPos[0]);
	mBall->setPosition(4, mBallGenerator->mInitPos[1]);
	mBall->setPosition(5, last_motion[5] + mBallGenerator->mInitPos[2]);
	mBall->setVelocity(3, mBallGenerator->mInitVel[0]);
	mBall->setVelocity(4, mBallGenerator->mInitVel[1]);
	mBall->setVelocity(5, mBallGenerator->mInitVel[2]);	
	mWorld->addSkeleton(mBall);	

	this->water_flag = true;
}

bool MyWindow::ballHeadCollision()
{
	auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
	auto shoulder_collision = collisionEngine->createCollisionGroup();
	auto head_collision = collisionEngine->createCollisionGroup();
	shoulder_collision->addShapeFramesOf(mHubo->getBodyNode("rclavicle"));
	shoulder_collision->addShapeFramesOf(mHubo->getBodyNode("lclavicle"));
	shoulder_collision->addShapeFramesOf(mBall.get());
	head_collision->addShapeFramesOf(mHubo->getBodyNode("head"));
	head_collision->addShapeFramesOf(mBall.get());
	bool collision = head_collision->collide() || shoulder_collision->collide();

	return collision;
}

void MyWindow::motionblend_init()
{
	mCurrentMotionblender = std::make_unique<MotionBlender>(mHubo->getPositions(), mMotions[0]);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
		//Implement Here 
		case 'q':
		this -> mCartoonFlag = !mCartoonFlag;
		break;
		// case 'e':
		// break;
		case 'r':
		this -> water_flag = !water_flag;
		break;
		case 't':
		cnt = 0;
		mMotions = mbvh->expMotionGetter();
		this -> motionblend_init();
		bvh_flag = true;
		break;
		case 'a':
		cnt = 0;
		mMotions = mMotionGraph-> run_then_jump();
		this -> motionblend_init();
		mBallGenerator->throw_target_default(5.0);
		mBallGenTime = mBallGenerator->mTimeStep * 1000;
		bvh_flag = true;
		break;
		case 's':
		cnt = 0;
		mMotions = mMotionGraph-> walk_then_jump();
		this -> motionblend_init();
		mBallGenerator->throw_target_default(3.0);
		mBallGenTime = mBallGenerator->mTimeStep * 1000;
		bvh_flag = true;
		break;
		default:
		SimWindow::keyboard(key, x, y);
	}	
}


// Simulation Time Stepping Function
// This function is called every single step.
void MyWindow::timeStepping()
{
	//  BVH Following STep
	if(bvh_flag)
	{
		Eigen::VectorXd blend_current = mCurrentMotionblender->root_aligned_pose(mMotions[cnt]);
		mController->setTargetPosition(blend_current);
		if(mMotionGraph->mJumpTime - cnt == int(mBallGenTime))
		{
			if(water_flag)
				this->throw_water_ball();
			else
				this->throw_normal_ball();
		}
		cnt++;
		if(cnt == mMotions.size()){
			cnt = 0;
			bvh_flag = false;
			PD_flag = true;
		}
	}

	// Header, Exert force to a ball.
	if(this->ballHeadCollision())
	{
		mBall->getBodyNode(0)->setExtForce(Eigen::Vector3d(0,15,2));
		if(water_flag)
		{
			if(mCartoonFlag)
				mController->setAllZero();
			else
				mController->setRootZero();
		}
	}

	// PD_Control if it is possible
	if(PD_flag)
	{
		mController->clearForces();
		mController->addSPDForces();
	}


	Eigen::Vector3d skirtRoot_dir = (getJointTransform("tibial") + getJointTransform("tibiar"))/2 - getJointTransform("torso");
	Eigen::Vector3d skirtRoot = skirtRoot_dir/skirtRoot_dir.norm()*0.5 + getJointTransform("torso");

	mCloth->SetPosition(	getJointTransform("torso"),
						getJointTransform("head"),
						getJointTransform("upperarml"),
						getJointTransform("forearml"),
						getJointTransform("upperarmr"),
						getJointTransform("forearmr"),
						skirtRoot);

	mCloth->TimeStep();


	// Override timestepping from default dartsim world
	SimWindow::timeStepping();

}

// Drawing Function. Every SImWindow TimeStepping function calls this draw function
void MyWindow::draw()
{
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
	// glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (!mSimulating) {
		if (mPlayFrame < mWorld->getRecording()->getNumFrames()) {
			std::size_t nSkels = mWorld->getNumSkeletons();
			for (std::size_t i = 0; i < nSkels; i++) {
				mWorld->getSkeleton(i)->setPositions(mWorld->getRecording()->getConfig(mPlayFrame, i));
			}	      
			if (mShowMarkers) {
				int nContact = mWorld->getRecording()->getNumContacts(mPlayFrame);
				for (int i = 0; i < nContact; i++) {
					Eigen::Vector3d v = mWorld->getRecording()->getContactPoint(mPlayFrame, i);
					Eigen::Vector3d f = mWorld->getRecording()->getContactForce(mPlayFrame, i);

					glBegin(GL_LINES);
					glVertex3f(v[0], v[1], v[2]);
					glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
					glEnd();
					mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
					mRI->pushMatrix();
					glTranslated(v[0], v[1], v[2]);
					mRI->drawSphere(0.01);
					mRI->popMatrix();
				}
			}
		}
	} 
	else {glPointSize(10.0);  
		if (mShowMarkers) {
			const auto result =
			mWorld->getConstraintSolver()->getLastCollisionResult();
			for (const auto& contact : result.getContacts()) {
				Eigen::Vector3d v = contact.point;
				Eigen::Vector3d f = contact.force / 10.0;
				glBegin(GL_LINES);
				glVertex3f(v[0], v[1], v[2]);
				glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
				glEnd();
				mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
				mRI->pushMatrix();
				glTranslated(v[0], v[1], v[2]);
				mRI->drawSphere(0.01);
				mRI->popMatrix();
			}
		}
	}

	glDisable(GL_LIGHTING);
	// Implement 2D factors inside here


	glEnable(GL_LIGHTING);
	// Implement 3D factors inside here
	drawSphere(0.1, 10 , 10, getJointTransform("torso"));
	drawSphere(0.1, 10 , 10, getJointTransform("head"));
	drawSphere(0.1, 10 , 10, getJointTransform("upperarml"));
	drawSphere(0.1, 10 , 10, getJointTransform("forearml"));
	drawSphere(0.1, 10 , 10, getJointTransform("upperarmr"));
	drawSphere(0.1, 10 , 10, getJointTransform("forearmr"));

	Eigen::Vector3d skirtRoot_dir = (getJointTransform("tibial") + getJointTransform("tibiar"))/2 - getJointTransform("torso");
	Eigen::Vector3d skirtRoot = skirtRoot_dir/skirtRoot_dir.norm()*0.5 + getJointTransform("torso");

	drawSphere(0.1, 10 , 10, skirtRoot);
	
	std::cout << "==================" << std::endl;
	std::cout << getJointTransform("torso").transpose() << std::endl;
	std::cout << getJointTransform("head").transpose() << std::endl;
	std::cout << getJointTransform("upperarml").transpose() << std::endl;
	std::cout << getJointTransform("forearml").transpose() << std::endl;
	std::cout << getJointTransform("upperarmr").transpose() << std::endl;
	std::cout << getJointTransform("forearmr").transpose() << std::endl;
	std::cout << skirtRoot.transpose() << std::endl;

	// mCloth->SetPosition(	getJointTransform("torso"),
	// 					getJointTransform("head"),
	// 					getJointTransform("upperarml"),
	// 					getJointTransform("forearml"),
	// 					getJointTransform("upperarmr"),
	// 					getJointTransform("forearmr"),
	// 					skirtRoot);

	// mCloth->TimeStep();

	draw_cloth(mCloth->getVertices());


	// Draw rest world components
	drawWorld();

	// display the frame count in 2D text
	char buff[64];
	if (!mSimulating)
	#ifdef _WIN32
		_snprintf(buff, sizeof(buff), "%d", mPlayFrame);
	#else
	std::snprintf(buff, sizeof(buff), "%d", mPlayFrame);
	#endif
	else
	#ifdef _WIN32
		_snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
	#else
	std::snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
	#endif
	std::string frame(buff);
	glColor3f(0.0, 0.0, 0.0);
	drawStringOnScreen(0.02f, 0.02f, frame);
	glEnable(GL_LIGHTING);
}