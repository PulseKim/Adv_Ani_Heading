#include "MyWindow.h"

MyWindow::MyWindow(const WorldPtr& world) : SimWindow()
{
	PD_flag = true;
	this->setWorld(world);
	this->initParameters();
	this->initWindowSetting();
}


void MyWindow::initParameters()
{
	mWorld->setGravity(Eigen::Vector3d(0, -9.81, 0));
	rad_obj = 0.04;
	height_obj = 0.15;
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
	if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet"))
  	{
    	mWorld->getConstraintSolver()->setCollisionDetector(
    	  dart::collision::CollisionDetector::getFactory()->create("bullet"));
  	}
	this->initSkeleton();
	this->setSkeleton();
	this->addSkeleton();
	mController = std::make_unique<Controller>(mHubo);
}

void MyWindow::initSkeleton()
{
	this->loadHuboFloor();
}

void MyWindow::setSkeleton()
{
	Eigen::VectorXd default_pose = mHubo->getPositions();
	default_pose[4] = 0.9;
	mHubo->setPositions(default_pose);

	// Visual Aspect
	auto visualShapenodes = mFloor->getBodyNode(0)->getShapeNodesWith<VisualAspect>();
	// std::cout << visualShapenodes[0]->getVisualAspect()->getColor() << std::endl;s
	// visualShapenodes[0]->getVisualAspect()->setColor(dart::Color::Red(1.0));
}

void MyWindow::addSkeleton()
{
	mWorld->addSkeleton(mHubo);
	mWorld->addSkeleton(mFloor);
}

std::string MyWindow::GetCurrentWorkingDir() 
{
  char buff[FILENAME_MAX];
  GetCurrentDir( buff, FILENAME_MAX );
  std::string current_working_dir(buff);
  return current_working_dir;
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
		//Implement Here 
		case 'q':
		PD_flag = !PD_flag;
		break;
		default:
		SimWindow::keyboard(key, x, y);
	}	
}


void MyWindow::timeStepping()
{
	if(PD_flag)
	{
		mController->clearForces();
		mController->addSPDForces();
	}
	SimWindow::timeStepping();
	// mController->setFreeJointPosition();
}

void MyWindow::draw()
{
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
	// glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (!mSimulating) {
		if (mPlayFrame < mWorld->getRecording()->getNumFrames()) {
			std::size_t nSkels = mWorld->getNumSkeletons();
			for (std::size_t i = 0; i < nSkels; i++) {
	        // std::size_t start = mWorld->getIndex(i);
	        // std::size_t size = mWorld->getSkeleton(i)->getNumDofs();
				mWorld->getSkeleton(i)->setPositions(mWorld->getRecording()->getConfig(mPlayFrame, i));
			}	      
			if (mShowMarkers) {
	        // std::size_t sumDofs = mWorld->getIndex(nSkels);
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

	// this->drawNormals();
	// glDisable(GL_LIGHTING);
	// //Implement 2D factors inside here
	// this->drawFrame();
	glEnable(GL_LIGHTING);
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

// void MyWindow::drawNormals()
// {
// 	glEnable(GL_LIGHTING);
// 	this->showDirection(true, Eigen::Vector3d(0,0.15,0), Eigen::Vector3d(0,-0.1,0));
// }

// Eigen::VectorXd MyWindow::ForwardKinematicsMovement(int current_idx, int total_steps, const Eigen::VectorXd original, const Eigen::VectorXd target)
// {
// 	Eigen::VectorXd pose = original;
// 	for(int j = 0 ; j < target.size(); ++j){
// 		pose[j] = original[j] + (target[j] - original[j]) * current_idx / total_steps;
// 	}
// 	return pose;
// }


// void MyWindow::drawFrame()
// {
// 	dart::dynamics::Frame *hand_frame = mHand->getBodyNode("patch0");
// 	Eigen::Matrix3d rot = hand_frame->getWorldTransform().rotation();
// 	Eigen::Vector3d trans = hand_frame->getWorldTransform().translation();
// 	glLineWidth(3.0);
// 	for(int i = 0; i < 3; i++){
// 		Eigen::Vector3d current_axis = rot.col(i).normalized() * 0.1;
// 		glColor3f(0.0,i * 0.5,0.0);
// 		glBegin(GL_LINES);
// 		glVertex3f(trans[0], trans[1], trans[2]);
// 		glVertex3f(trans[0] + current_axis[0], trans[1] + current_axis[1], trans[2] + current_axis[2]);
// 		glEnd();
// 	}
// }

// //Depricated Currently
// void MyWindow::showDirection(bool flag, Eigen::Vector3d begin, Eigen::Vector3d dir)
// {
// 	if(!flag) return;
// 	dir.normalize();
// 	dir = 0.03 * dir;
// 	glColor3f(0.0, 1.0, 1.0); 
// 	glLineWidth(4.0);
// 	glBegin(GL_LINES);
// 	glVertex3f(begin[0], begin[1], begin[2]);
// 	glVertex3f(begin[0] + dir[0], begin[1] + dir[1], begin[2] + dir[2]);
// 	glEnd();
// 	mRI->setPenColor(Eigen::Vector3d(0.0, 1.0, 1.0));
// 	mRI->pushMatrix();
// 	glTranslated(begin[0], begin[1], begin[2]);
// 	mRI->drawSphere(0.005);
// 	mRI->popMatrix();

// }


// void MyWindow::tempCollision()
// {
// 	// std::cout << mWorld->getSimFrames() << std::endl;
// 	auto cont = mWorld->getLastCollisionResult().getContacts();
// 	// std::cout << cont[0].force <<std::endl;
// 	// std::cout << mWorld->getLastCollisionResult().getContact(0).force << std::endl;
// 	// for(int i= 0; i < cont.size(); ++i){
// 	// 	std::cout << mWorld->getLastCollisionResult().getContact(i).collisionObject1->getShapeFrame()->getName() <<std::endl;
// 	// 	std::cout << mWorld->getLastCollisionResult().getContact(i).collisionObject2->getShapeFrame()->getName() <<std::endl;
// 	// }
// 	// auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
// 	// auto objGroup = collisionEngine->createCollisionGroup(mObj.get());
// 	// auto finger1 = collisionEngine->createCollisionGroup((mArm_r->getBodyNode("distphalanx2")));
// 	// auto handGroup = collisionEngine->createCollisionGroup();
// 	// handGroup->addShapeFramesOf(mArm_r->getBodyNode("thumb distphalanx"));	
// 	// handGroup->distance(objGroup.get());
// 	// handGroup->removeShapeFramesOf(mArm_r->getBodyNode("thumb distphalanx"));
// 	// for(int i =0 ; i < 4;++i){
// 	// 	handGroup->addShapeFramesOf(mArm_r->getBodyNode("distphalanx"+ std::to_string(i)));
// 	// 	std::cout <<"collide" << handGroup->collide(finger1.get()) << std::endl;
// 	// 	handGroup->removeShapeFramesOf(mArm_r->getBodyNode("distphalanx"+ std::to_string(i)));
// 	// }
// }
