#include "MotionGraph.h"


MotionGraph::MotionGraph(SkeletonPtr &biped, float time_step)
{
	mTimeStep = time_step;
	mHubo = biped;
	this -> walk();
	this -> jump();
	this -> run();
}

std::vector<Eigen::VectorXd> MotionGraph::walk_then_jump()
{
	auto blender = std::make_unique<MotionBlender>(mWalk[mWalk.size()-1], mJump[0]);
	std::vector<Eigen::VectorXd> interpolate;
	
}

void MotionGraph::walk()
{
	std::string file_name = "16_31_walk.bvh";
	auto mbvh = std::make_unique<bvh>(file_name, mHubo, mTimeStep);
	mWalk = mbvh -> expMotionGetter();
 }

void MotionGraph::jump()
{
	std::string file_name = "16_01_jump.bvh";
	auto mbvh = std::make_unique<bvh>(file_name, mHubo, mTimeStep);
	mJump = mbvh -> expMotionGetter();

}

void MotionGraph::run()
{
	std::string file_name = "16_46_run&jog.bvh";
	auto mbvh = std::make_unique<bvh>(file_name, mHubo, mTimeStep);
	mRun = mbvh -> expMotionGetter();

}

float MotionGraph::walk_dist()
{
	Eigen::VectorXd start = mWalk[0];
	Eigen::VectorXd end = mWalk[mWalk.size()-1];
	float dist =  float(abs(start[5] - end[5])) / 5.0;
	return dist;
}

float MotionGraph::run_dist()
{
	Eigen::VectorXd start = mRun[0];
	Eigen::VectorXd end = mRun[mRun.size()-1];
	float dist =  float(abs(start[5] - end[5])) / 4.0;
	return dist;
}

float MotionGraph::jump_height()
{
	float start = mJump[0][4];
	float dist = 0;
	for(int i = 1; i < mJump.size(); ++i)
	{
		float curr = mJump[i][4] - start;
		if(curr > dist)
			dist = curr;
	}	
	return dist;
}