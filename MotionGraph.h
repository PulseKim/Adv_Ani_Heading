#ifndef _MOTIONGRAPH_H_
#define _MOTIONGRAPH_H_

#include <iostream>
#include "dart/dart.hpp"
#include "bvh.h"
#include "MotionBlender.h"

using namespace dart::dynamics;


class MotionGraph
{
public:
	MotionGraph(SkeletonPtr &biped, float time_step);
	std::vector<Eigen::VectorXd> walk_then_jump();
	std::vector<Eigen::VectorXd> run_then_jump();
	std::vector<Eigen::VectorXd> smoothTransition(Eigen::VectorXd &current, Eigen::VectorXd &future, int blend_step);

	void walk();
	void jump();
	void run();
	float walk_dist();
	float run_dist();
	float jump_height();


private:
	float mTimeStep;
	SkeletonPtr mHubo;
	std::vector<Eigen::VectorXd> mWalk;
	std::vector<Eigen::VectorXd> mRun;
	std::vector<Eigen::VectorXd> mJump;
};

#endif
