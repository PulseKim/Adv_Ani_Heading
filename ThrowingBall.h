#ifndef _THROWINGBALL_H_
#define _THROWINGBALL_H_

#include "dart/dart.hpp"
#include "MotionGraph.h"
using namespace dart::dynamics;

class Throw
{
public:
	Throw(SkeletonPtr &hubo, float time);
	// ~Throw();

	void throw_target_default(float target_height);


public:
	float mParallelVel;
	float mTimeStep;
	float mStartY;
	Eigen::Vector3d mInitVel;
	Eigen::Vector3d mInitPos;

protected:
	std::unique_ptr<MotionGraph> mMotionGraph;
	SkeletonPtr mHubo;


};

#endif