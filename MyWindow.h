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
#include "bvh.h"
#include "MotionBlender.h"
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

	void throw_ball();

	// //Overriding functions
	void keyboard(unsigned char key, int x, int y) override;
	void timeStepping() override;
	void draw() override;

	// //Useful functions
	// Eigen::VectorXd ForwardKinematicsMovement(int current_idx, int total_steps, const Eigen::VectorXd original, const Eigen::VectorXd target);
	// void playSavedMovement(std::string filePath);
	// double degToRad(double degree);
	// void tempCollision();
	// void testing();
	// void drawFrame();
	// void drawNormals();
	// void showDirection(bool flag, Eigen::Vector3d begin, Eigen::Vector3d dir);

protected:
	bool bvh_flag;
	bool PD_flag;
	SkeletonPtr	mFloor;
	SkeletonPtr mHubo;
	SkeletonPtr mBall;
	std::unique_ptr<Controller> mController;
	std::vector<Eigen::VectorXd> mMotions;
	std::unique_ptr<bvh> mbvh;
	std::unique_ptr<MotionBlender> mCurrentMotionblender;
public:
	int cnt;
	int motion_count;
	int motion_peak;
};

#endif