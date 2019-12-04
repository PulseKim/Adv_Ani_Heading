#include <iostream>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/utils/utils.hpp"
#include <unistd.h>
#define GetCurrentDir getcwd
#include "Controller.h"
#include "SkelGen.h"
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
	bool PD_flag;
	SkeletonPtr	mFloor;
	SkeletonPtr mHubo;
	SkeletonPtr mBall;
	std::unique_ptr<Controller> mController;

public:
	double rad_obj;
	double height_obj;
};
