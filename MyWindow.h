#ifndef _MYWINDOW_H_
#define _MYWINDOW_H_

#include <iostream>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/utils/utils.hpp"
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <unistd.h>
#define GetCurrentDir getcwd
#include "Controller.h"
#include "SkelGen.h"
#include "MotionGraph.h"
#include "ThrowingBall.h"
// #include "IkSolver.h"


using namespace dart::simulation;
using namespace dart::gui::glut;
using namespace dart::dynamics;
using namespace dart::utils;

class MyWindow: public SimWindow
{
public:
	MyWindow(const WorldPtr& world);

	//Initialization
	void initParameters();
	void loadHuboFloor();
	void initWindowSetting();
	void initSkeleton();
	void setSkeleton();
	void addSkeleton();
	std::string GetCurrentWorkingDir(); 
	void motionblend_init();

	void throw_normal_ball();
	void throw_water_ball();
	bool ballHeadCollision();

	// //Overriding functions
	void keyboard(unsigned char key, int x, int y) override;
	void timeStepping() override;
	void draw() override;
	

protected:
	bool mCartoonFlag;
	bool bvh_flag;
	bool water_flag;
	bool PD_flag;
	SkeletonPtr	mFloor;
	SkeletonPtr mHubo;
	SkeletonPtr mBall;
	std::unique_ptr<Controller> mController;
	std::vector<Eigen::VectorXd> mMotions;
	std::unique_ptr<bvh> mbvh;
	std::unique_ptr<MotionBlender> mCurrentMotionblender;
	std::unique_ptr<MotionGraph> mMotionGraph;
	std::unique_ptr<Throw> mBallGenerator;
	float mBallGenTime;
public:
	int cnt;
	int motion_count;
	int motion_peak;

};

#endif