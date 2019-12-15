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
#include "FastMassSpring/sim/Cloth.h"
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
	

	// test draw
	void drawSphere(double r, int lats, int longs, Eigen::Vector3d center) {
    int i, j;
    for(i = 0; i <= lats; i++) {
        double lat0 = M_PI * (-0.5 + (double) (i - 1) / lats);
        double z0  = sin(lat0);
        double zr0 =  cos(lat0);

        double lat1 = M_PI * (-0.5 + (double) i / lats);
        double z1 = sin(lat1);
        double zr1 = cos(lat1);

        glBegin(GL_QUAD_STRIP);
        for(j = 0; j <= longs; j++) {
            double lng = 2 * M_PI * (double) (j - 1) / longs;
            double x = cos(lng);
            double y = sin(lng);

            glNormal3f(x * zr0, y * zr0, z0);
            glVertex3f(center(0) + r * x * zr0,center(1) + r * y * zr0, center(2) + r * z0);
            glNormal3f(x * zr1, y * zr1, z1);
            glVertex3f(center(0) + r * x * zr1, center(1) + r * y * zr1, center(2) + r * z1);
        }
        glEnd();
    	}
	}


	Eigen::Vector3d getJointTransform(const std::string &name)
	{
		if (name=="torso")
		{
			return mHubo->getBodyNode(0)->getTransform().translation();
		}
		else
		{
			Joint* curJoint = mHubo->getJoint(name);
			BodyNode* ParentBN = curJoint->getParentBodyNode();
			Eigen::Isometry3d parentBNTransform = ParentBN->getTransform();
			Eigen::Vector3d offset_b =  curJoint->getTransformFromParentBodyNode().translation();
			Eigen::Vector3d jointH = parentBNTransform.translation() + parentBNTransform.linear() * offset_b;
			return jointH;
		}
	}

	void draw_triangle(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3){
		glBegin(GL_TRIANGLES);
		// glColor3f(0.1, 0.2, 0.3);
		glVertex3f(v1(0), v1(1), v1(2));
		glVertex3f(v2(0), v2(1), v2(2));
		glVertex3f(v3(0), v3(1), v3(2));
		glEnd();
	}

	void draw_quad(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3, Eigen::Vector3d v4){
		glBegin(GL_QUADS);
		// glColor3f(0.1, 0.2, 0.3);
		glVertex3f(v1(0), v1(1), v1(2));
		glVertex3f(v2(0), v2(1), v2(2));
		glVertex3f(v3(0), v3(1), v3(2));
		glVertex3f(v4(0), v4(1), v4(2));
		glEnd();
	}

	void draw_cloth(std::vector<Eigen::Vector3d> vs, string type = "quad")
	{
		if (type == "tri")
		{
			size_t tri_num = vs.size()/3;
			for(size_t i = 0; i < tri_num; i++) { draw_triangle( vs[3*i], vs[3*i+1], vs[3*i+2] ); }
		}
		else if (type == "quad")
		{
			size_t tri_num = vs.size()/4;
			for(size_t i = 0; i < tri_num; i++) { draw_quad( vs[4*i], vs[4*i+1], vs[4*i+2], vs[4*i+3] ); }
		}
	}


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
	std::unique_ptr<Cloth> mCloth;
	float mBallGenTime;
public:
	int cnt;
	int motion_count;
	int motion_peak;

};

#endif